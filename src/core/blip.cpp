// 
//  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// 

#include "blip.h"
#include "blip_p.h"

#include <QtCore/QMutexLocker>
#include <QtCore/QTime>

#include <cmath>

#include "Nodes/node.h"
#include "Nodes/node_p.h"
#include "Graph/graphsupport.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsScene>
  #include <QtWidgets/QGraphicsItem>
  
  #include "Graphics/palette.h"
  #include "Graphics/geometryitem.h"
  #include "IDL/idlsupport.h"
#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  BlipPrivate                                                               //
//                                                                            //
//############################################################################//

BlipPrivate::BlipPrivate ( Blip* q )
  : q_ptr ( q )
{//NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!

    
    auto zero_2d = Geometry::Point_2D(0.0,0.0); //FIXME: Magic Numbers
    auto zero_3d = AMG::Vector(0,0,0, AMG::CoSy::getDatumFrame()); //FIXME::Magic Numbers
    
    
    modeDescriptor.ensure_length(1,1);
//     modeDescriptor[0] = IDL::Data::STATE_ID_NOT_APPLICABLE;
    
    // NOTE: When a Blip(Private) is instantiated it is either the ownship or a newly
    // discovered other blip. In both cases, STATE_ID_D_LINK_UP is true by 
    // default.
    modeDescriptor[0] = IDL::Data::STATE_ID_D_LINK_UP;
   
    
    
    position3D = zero_3d;
    position2D = zero_2d;
    attitude.setZero();
    velocity = zero_3d;
    
    
    /** \todo An intialization to zero_2d can causes an issue for newly 
     * initialized Blips: if the Blip is exactly at (or really) close to the 
     * datum, i.e. the (0,0) position, then the R/F polygon won't be computed 
     * as the minimum distance threshold won't be crossed. This isn't an issue 
     * for moving blips, but can prevent a GCS blip (which is presumably both 
     * close to the Datum as well as potentially not moving) from ever 
     * generating a R/F polygong other than the the default small ellipse... 
     * effectively preventing such a Blip from ever connecting to the swarm. */
    
    auto offset_2d = Geometry::Point_2D(10.0,10.0); //FIXME: Magic Numbers
    boost::geometry::assign(rfPolygonCenter,offset_2d );
    
    auto circle = Geometry::makeCircle<Geometry::Polygon_2D::ring_type,100>(rfPolygonCenter,10); //FIXME: Magig Numbers
    
    boost::geometry::assign(rfPolygon.outer(),circle );
    boost::geometry::assign(lastWpRfPolygon.outer(),circle );
    
    boost::geometry::assign(visPolygonCenter, zero_2d);
    boost::geometry::assign(visPolygon.outer(), circle );
    
    boost::geometry::assign(fovPolygonCenter, zero_2d);
    boost::geometry::assign(fovPolygon.outer(), circle );
//     comGraphVertex = NULL; //TODO: initialize this to NULL?

    currentFlightplan.waypoints.length(0);
    
#ifdef WITH_GRAPHICS
  {
    using namespace Graphics_2D;

    rfPolygonItem = new Polygon(rfPolygon);
    rfPolygonItem->setPen(Pens::Outline());
    rfPolygonItem->setBrush(Qt::NoBrush); 
    
    lastWpRfPolygonItem = new Polygon(lastWpRfPolygon);
    lastWpRfPolygonItem->setPen(Pens::Outline(Qt::gray));
    lastWpRfPolygonItem->setBrush(Qt::NoBrush);
    lastWpRfPolygonItem->hide();
    
    fovPolygonItem = new Polygon(fovPolygon);
    //     fovPolygonItem->hide();
    
    rfPolygonCenterItem  = new QGraphicsEllipseItem(-2,-2,4,4); //FIXME: Magic Numbers //TODO: make this match the updateRfPoly minDistance.
    rfPolygonCenterItem->setPen(Pens::Outline());
    rfPolygonCenterItem->setBrush(Qt::NoBrush); 
    
    
    bodyCarriedFrameItem = new CoordinateSystem;
    bodyCarriedFrameItem->hideAxes(); //NOTE: don't hide the complete item as that would hide all children, too.
    
    bodyFrameItem = new CoordinateSystem(bodyCarriedFrameItem);
    bodyFrameItem->hideAxes(); //NOTE: don't hide the complete item as that would hide all children, too.
    
    velocityItem = new Graphics_2D::Arrow(bodyCarriedFrameItem);
    velocityItem->setRootPosition(QPointF(0,0));
    velocityItem->setHeadPosition(QPointF(1,0));
    velocityItem->setHeadSize(2); //FIXME: Magic Numbers
    velocityItem->setColor(Qt::yellow);
    velocityItem->hide();
     
//     currentWaypointItem; // this item gets deleted and recreated with every call to updateCurrentWaypointGraphics
  } 
#endif // WITH_GRAPHICS
}

BlipPrivate::~BlipPrivate()
{
#ifdef WITH_GRAPHICS
  delete bodyCarriedFrameItem;
  bodyCarriedFrameItem = nullptr;
  
  //NOTE: as these are children of bodyCarriedFrameItem, they are deleted with 
  // their parent item.
//   delete velocityItem;
//   velocityItem = nullptr;
//   
//   delete bodyFrameItem;
//   bodyFrameItem = nullptr;
  
  delete rfPolygonCenterItem;
  rfPolygonCenterItem = nullptr;
  
  delete lastWpRfPolygonItem;
  lastWpRfPolygonItem = nullptr;
  
  delete rfPolygonItem;
  rfPolygonItem = nullptr;
  
  delete fovPolygonItem;
  fovPolygonItem = nullptr;
  
  delete currentFlightplanItem;
  currentFlightplanItem = nullptr;
  
  delete currentWaypointItem;
  currentFlightplanItem = nullptr;
  
  datumItem->update();
  datumItem = nullptr;
  
#endif // WITH_GRAPHICS
}

#ifdef WITH_GRAPHICS


QAbstractGraphicsShapeItem* BlipPrivate::generateWaypointGraphics(IDL::Data::Waypoint const & waypoint, bool active, qreal radius)
{
  return generateWaypointGraphics( QPointF(waypoint.position.x,waypoint.position.y)
                                 , active, radius);
}

QAbstractGraphicsShapeItem* BlipPrivate::generateWaypointGraphics(QPointF position2D, bool active, qreal radius)
{ 
  auto wpItem = new QGraphicsEllipseItem(position2D.x(),position2D.y()
                                        ,2* radius,2*radius);
  wpItem->moveBy(-radius,-radius);
  
  QColor color;
  if(active)
  { color = Graphics_2D::Colors::activeFlightplan; }
  else
  { color = Graphics_2D::Colors::inactiveFlightplan; }
  
  wpItem->setBrush(Graphics_2D::Brushes::FloodFill(color));
  wpItem->setPen(Graphics_2D::Pens::Outline(color));
  
  return wpItem;
}

void BlipPrivate::updatePalette()
{
  Q_Q(Blip); Q_ASSERT(q);
  
  Node* parentNode = q->parent();
  Q_ASSERT(parentNode);
  
  bool blipRepresentsOwnship = (q->id() == parentNode->id());
  
  auto useUtensils = [](Graphics_2D::Utensils const & utensils, QAbstractGraphicsShapeItem* shape)
  {
    shape->setPen(utensils.pen);
    shape->setBrush(utensils.brush);
    shape->update();   
  };
  
  using namespace IDL::Data;
  
  if( q->isConnectedToHost() )
  {
    /** \internal Compare statesToDraw with Node::statesToReport */
    
    QList<StateDescriptorId> statesToDraw; //FIXME: Magic Numbers: States that cause an altered drawing of the RfPolygon.
    
    //NOTE: Order matters when inserting states! Substates have to be entered 
    // _before_ their superstates!
    statesToDraw
      << STATE_ID_N_STANDBY
      
      << STATE_ID_V_JOYSTICK //NOTE: JOYSTICK is a substate of PILOTED and hence has to be inserted _before_ PILOTED 
      << STATE_ID_V_PILOTED
      
      << STATE_ID_N_JOINING
      << STATE_ID_N_LEAVING
      << STATE_ID_N_OPERATIONAL
//    << STATE_ID_V_AUTONOMOUS 
      ;
    
    
    for( StateDescriptorId & state : statesToDraw)
    {
     if( mode.contains(state) )
     { 
//        qDebug() << q->parent()->qualifiedName() << QTime::currentTime() 
//        << ": updating Blip palette for" << q->name() << "with state" 
//        << state;
       
       if( blipRepresentsOwnship )
       { Q_ASSERT(palette->rfPoly_ownship.contains(state));
         useUtensils( palette->rfPoly_ownship[state], rfPolygonItem); }
       else if( isRfRelay )
       { Q_ASSERT(palette->rfPoly_relay.contains(state));
         useUtensils( palette->rfPoly_relay[state], rfPolygonItem); }
       else
       { Q_ASSERT(palette->rfPoly_nonRelay.contains(state));
         useUtensils( palette->rfPoly_nonRelay[state], rfPolygonItem); }
       break;
     }
    }
  }
  else // not connected to host
  {
    if( blipRepresentsOwnship )
    { useUtensils(palette->rfPoly_ownship[STATE_ID_D_NO_LINK], rfPolygonItem); }
    else
    { useUtensils(palette->rfPoly_nonRelay[STATE_ID_D_NO_LINK], rfPolygonItem);}
  }
  
  if( q->isJoystickControlled() )
  { useUtensils(palette->fovPoly_joystick, fovPolygonItem); }
  else
  { useUtensils(palette->fovPoly_nonJoystick, fovPolygonItem); }
    

}

void BlipPrivate::updateCurrentFlightplanGraphics(int const & futureWpCount, int const & pastWpCount)
{
  delete currentFlightplanItem;
  currentFlightplanItem = nullptr;
  
  currentFlightplanItem = new QGraphicsItemGroup; 
 
  uint numberOfWaypoints = currentFlightplan.waypoints.length();
  
  // The number of future (next) waypoints, i.e. after the currentWaypoint, that 
  // should be drawn and connected by a line when plotting the flightplan.
  uint futureWpsToDraw = futureWpCount</*draw all Wps*/0  ? numberOfWaypoints : static_cast<uint>(futureWpCount);
  
  // The number of past (passed) waypoints, i.e. before the currentWaypoint, 
  // that shoulb be drawn when plotting the flightplan.
  uint pastWpsToDraw = pastWpCount</*draw all Wps*/0 ?  numberOfWaypoints : static_cast<uint>(pastWpCount);
  

  if(numberOfWaypoints > 0)
  {
    if( isConnectedToHost and currentFlightplanIsValid)
    {
      //NOTE: be carefull in the subtraction as those are uints (=> overrun)
      uint firstWpIndex = currentWaypointIndex>pastWpsToDraw ? currentWaypointIndex-pastWpsToDraw : 0;
      uint lastWpIndex = qMin(numberOfWaypoints-1,currentWaypointIndex+futureWpsToDraw);
        
      IDL::Data::Waypoint wp = currentFlightplan.waypoints[firstWpIndex];
      Geometry::Point_2D tail;
      tail.x(wp.position.x);
      tail.y(wp.position.y);
      
      //draw the first waypoint
      currentFlightplanItem->addToGroup(
        generateWaypointGraphics(wp,/*bool active*/firstWpIndex>=currentWaypointIndex)
      );
      
      for(uint i=firstWpIndex+1; i<=lastWpIndex; ++i)
      { 
        wp = currentFlightplan.waypoints[i];

        Geometry::Point_2D head;
        head.x(wp.position.x);
        head.y(wp.position.y);
        


        if( i>currentWaypointIndex )
        {
          auto line = new QGraphicsLineItem(tail.x(), tail.y(),
                                            head.x(), head.y());
          
          line->setPen(Graphics_2D::Pens::Outline(Graphics_2D::Colors::activeFlightplan));
          
          currentFlightplanItem->addToGroup(line);
        }
        
        currentFlightplanItem->addToGroup(
          generateWaypointGraphics(wp,/*bool active*/i>=currentWaypointIndex)
        );
        
        tail = head;
      }

    }
    
    //
    // Create a special graphical representation of the last Waypoint
    //
 
    lastWpItem = generateWaypointGraphics(currentFlightplan.waypoints[numberOfWaypoints-1]
      ,/*active*/true, /*radius*/ 1.5);
    lastWpItem->setPen(QPen(mainColor,1));
        
    currentFlightplanItem->addToGroup(lastWpItem);    
  }

  currentFlightplanItem->setParentItem(datumItem);
  currentFlightplanItem->setZValue(NodePrivate::BlipCurrentFlightplanZLevel);
 
}

void BlipPrivate::updateCurrentWaypointGraphics()
{
  delete currentWaypointItem;
  currentWaypointItem = nullptr;

  currentWaypointItem = new QGraphicsItemGroup;
  
  if( !isConnectedToHost or !currentFlightplanIsValid)
  { return; }
  
  DDS_Long fp_length = currentFlightplan.waypoints.length();
  
  if( /*!mode.contains(IDL::Data::STATE_ID_D_NO_LINK) and */
    static_cast<DDS_Long>(currentWaypointIndex) < fp_length ) //NOTE: fixing a signed-vs.-unsigned issue
  {
    
    IDL::Data::Waypoint currentWp = currentFlightplan.waypoints[currentWaypointIndex];

    auto line = new QGraphicsLineItem( position2D.x(), position2D.y(),
                                    currentWp.position.x, currentWp.position.y );
    
    line->setPen(Graphics_2D::Pens::Outline(Graphics_2D::Colors::activeFlightplan));

    currentWaypointItem->addToGroup(line);
  }
  
  currentWaypointItem->setParentItem(datumItem);
  currentWaypointItem->setZValue(NodePrivate::BlipCurrentWaypointZLevel);
}




QVariant BlipPrivate::modelData(  const int& column, int role ) const
{
  Q_Q(const Blip); Q_ASSERT(q);
  
  using da = TreeItem::DataAspect;
  
  if( role == Qt::DisplayRole )
  {
    switch( column )
    {
      case static_cast<int>(da::name) :     return QVariant(q->objectName());
      case static_cast<int>(da::value) :    return QVariant(generateModeDescription());
      case static_cast<int>(da::comment) :  return QVariant( QString("GUID : %1").arg(nodeId.toQString()) );
      case static_cast<int>(da::type) :     return QVariant( enumToQString(type) );
      default: return QVariant();
    }
  }
  else if( role == Qt::DecorationRole and column == 0)
  { return q->mainColor(); }
  else if( role == Qt::UserRole )
  { 
    QByteArray byteArray;
    byteArray =  nodeId.toByteArray();
    return QVariant(byteArray);
  }
  //else
  return QVariant();
}

QString BlipPrivate::generateModeDescription() const
{
 QStringList descriptor;
 
 for( auto state : mode )
 { descriptor << enumToQString(state); }
 
 return descriptor.join(", ");
}

void BlipPrivate::updatePersistentContextMenu()
{
  Q_Q(Blip); Q_ASSERT(q);
  rfPolyVisibilityAction->setChecked(q->isRfPolygonVisible());
  fovPolyVisibilityAction->setChecked(q->isFovPolygonVisible());
  bodyFrameVisibilityAction->setChecked(q->isBodyFrameVisible());
}

#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  ModelViewSubItems                                                         //
//                                                                            //
//############################################################################//
#ifdef WITH_GRAPHICS
BlipPrivate::ModelViewSubItems::ModelViewSubItems(BlipPrivate * const d)
{
  /** \internal
    * TreeItems don't assume ownership of the memory of their children. As such
    * they don't free it upon their deletion. As ModelViewSubItems calls new, it's the 
    * ModelViewSubItems's responsibilty to clean up. It does that in its destructor.
    */
  Blip* q = d->q_func(); Q_ASSERT(q);
  
  position = new PositionItem(d->position3D, QObject::tr("Position"),QObject::tr("relative to datum"));
  q->appendChild(position);

  orientation = new AttitudeItem(d->attitude, QObject::tr("Orientation"),QObject::tr("Body Euler Angles"));
  q->appendChild(orientation);

  speed = new VelocityItem(d->velocity, QObject::tr("Speed"), QObject::tr(""));
  q->appendChild(speed);

  debug = new BlipDebugItem(d);
  q->appendChild(debug);
}

BlipPrivate::ModelViewSubItems::~ModelViewSubItems()
{
  delete position;  position = nullptr;
  delete orientation; orientation = nullptr;
  delete speed; speed = nullptr;
  delete debug; debug = nullptr;
}

#endif //WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  Blip                                                                      //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Blip level construction needs to happen in Blip(BlipPrivate& dd, ...)!
 */
Blip::Blip(Node* parent)
  :Blip(*new BlipPrivate ( this ), parent)
{}

Blip::Blip(BlipPrivate & dd, Node* parent)
  :QObject(parent)
  ,d_ptr ( &dd )
{
#ifdef WITH_GRAPHICS
  {
    Q_D(Blip); Q_ASSERT(d);
    
    //NOTE: the blipItem can't be constructed in BlipPrivate() as it needs 
    // a Blip* pointer ("this") -- and "this" isn't contructed at that point yet.
    d->blipItem = new Graphics_2D::BlipItem(this,d->bodyFrameItem);
    
    connect(this, &QObject::objectNameChanged,
            [=](QString name) {
              d->blipItem->setToolTip(name);
              d->persistentContextMenu.setTitle(name);
            });
    
    //NOTE: this also cannot be constructed in BlipPrivate().
    d->mvSubItems = new BlipPrivate::ModelViewSubItems(d);
    
    
    connect(this, &Blip::rfRelayFlagChanged,
            [=](){d->updatePalette();} );
    
    connect(this, &Blip::connectednessToHostChanged,
            [=](){d->updatePalette();});
    
    //
    // Create the Blip's context menu
    //
    {
      QMenu* menu = &(d->persistentContextMenu);
      
      d->rfPolyVisibilityAction = new QAction(this);
      d->rfPolyVisibilityAction->setText( tr("&R/F Polygon"));
      d->rfPolyVisibilityAction->setCheckable(true);
      d->rfPolyVisibilityAction->setChecked(isRfPolygonVisible());
      menu->addAction(d->rfPolyVisibilityAction);
      
      connect( d->rfPolyVisibilityAction, &QAction::triggered,
              this, &Blip::setRfPolygonVisible);
      
      d->fovPolyVisibilityAction = new QAction(this);
      d->fovPolyVisibilityAction->setText( tr("&FOV Polygon"));
      d->fovPolyVisibilityAction->setCheckable(true);
      d->fovPolyVisibilityAction->setChecked(isFovPolygonVisible());
      menu->addAction(d->fovPolyVisibilityAction);
      
      connect(d->fovPolyVisibilityAction, &QAction::triggered,
              this, &Blip::setFovPolygonVisible);
      
      
      d->bodyFrameVisibilityAction = new QAction(this);
      d->bodyFrameVisibilityAction->setText( tr("&Body Frame"));
      d->bodyFrameVisibilityAction->setCheckable(true);
      d->bodyFrameVisibilityAction->setChecked(isBodyFrameVisible());
      menu->addAction(d->bodyFrameVisibilityAction);
      
      connect(d->bodyFrameVisibilityAction, &QAction::triggered,
              this,&Blip::setBodyFrameVisible );
      
    }
    
    
    /** \todo A HACK for swarm affiliation.
     * The "right" thing to do would be to incorporate a swarm ID into the 
     * status message of each Blip. A natural candidate would be the NodeId
     * of the cammandingControlStation of that Blip. */
    {
      connect(this, &Blip::modeChanged,
       [=](){
         if( mode().contains(IDL::Data::STATE_ID_N_STANDBY) )
         { d->rfPolygonCenterItem->setBrush(Graphics_2D::Brushes::FloodFill(Qt::lightGray)); ; }
         else
         { d->rfPolygonCenterItem->setBrush(Graphics_2D::Brushes::FloodFill(mainColor())); ; }
           
       });
    }
    
  }
#endif // WITH_GRAPHICS
}

Blip::~Blip()
{
  Q_D(Blip); Q_ASSERT(d);
#ifdef WITH_GRAPHICS
  {  
  delete d->mvSubItems;
  d->mvSubItems = nullptr;
  
  delete d->blipItem;
  d->blipItem = nullptr;
  }
#endif // WITH_GRAPHICS
}

Node* Blip::parent()
{ return static_cast<Node*>(QObject::parent()); }


NodeId Blip::id() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->nodeId;
}

void Blip::setId ( const NodeId& id )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->nodeId = id;
}


IDL::Data::NodeType Blip::type() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->type;
}

void Blip::setType ( const IDL::Data::NodeType& type )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->type = type;
}

QString Blip::name() const
{
  return objectName();
//   Q_D(const Blip); Q_ASSERT(d);
//   return d->name;
}

void Blip::setName ( const QString& name )
{
  setObjectName(name);

//   // QMutexLocker locker(&mutex);
//   Q_D(Blip); Q_ASSERT(d);
//   d->name = name;
}

AMG::Vector Blip::position3D() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->position3D;
}

void Blip::setPosition3D ( const AMG::Vector& position )
{
  Q_D(Blip); Q_ASSERT(d);

  d->position3D = position;
  
  // update the secondary data
  d->position2D.x( position.coords(0) );
  d->position2D.y( position.coords(1) );
  
  #ifdef WITH_GRAPHICS
  { 
    d->bodyCarriedFrameItem->setPos( d->position2D.x(), d->position2D.y() );
    d->updateCurrentWaypointGraphics();
  }
  {//HACK: MVC update hack
    // MVC: tell associated models that they need to update the views
    QAbstractItemModel* mvc_model =  model();
    if( mvc_model )
    {
      auto item = static_cast<TreeItem*>(d->mvSubItems->position);
      {//"value" of the position item
        auto top    =  mvc_model->index(item->childNumber(),static_cast<int>(DataAspect::value),index());
        auto bottom =  top;
        emit mvc_model->dataChanged(top,bottom);
      }
      { // x,y,z are the first three children of the position item.
        auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
        auto bottom =  mvc_model->index(2,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
        emit mvc_model->dataChanged(top,bottom);
      }
    }
  }
  #endif // WITH_GRAPHICS
    
  emit positionChanged();
}

Geometry::Point_2D Blip::position2D() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->position2D;
}

AMG::Vector Blip::velocity() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->velocity;
}

void Blip::setVelocity ( const AMG::Vector& velocity )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->velocity = velocity;
  d->speed = velocity.norm();
  d->heading_deg = AMG::Units::radian2degree(std::atan2(velocity.coords(1),velocity.coords(0)));
  
  #ifdef WITH_GRAPHICS
  { 
    if( d->speed > 0.1 ) //FIXME: Magic Numbers
    {
      d->velocityItem->setHeadPosition(QPointF(velocity.coords(0),velocity.coords(1)));
      d->velocityItem->show();
    }
    else
    { d->velocityItem->hide(); }
    
    {//HACK: MVC update hack
      // tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->speed);
        { // velocityItem
          auto top    =  mvc_model->index(item->childNumber(),static_cast<int>(DataAspect::value),index());
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
        { // xDot,yDot,zDot are the first three children of the position item.
          auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  mvc_model->index(2,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
    
  }
  #endif // WITH_GRAPHICS  
}

AMG::EulerAngleTupel Blip::attitude() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->attitude;
}

void Blip::setAttitude ( const AMG::EulerAngleTupel& eulerAngles )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->attitude = eulerAngles;
#ifdef WITH_GRAPHICS
  { 
    d->bodyFrameItem->setRotation(yaw());
    
    {//HACK: MVC update hack
      // tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->orientation);
        {//attitudeItem
          auto top    =  mvc_model->index(item->childNumber(),static_cast<int>(DataAspect::value),index());
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
        { // phi,theta,psi are the first three children of the position item.
          auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  mvc_model->index(2,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
  }
#endif // WITH_GRAPHICS
}

double Blip::altitude() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return -1.0*d->position3D.coords(2); //NOTE: the -1 is to correct for the Datum being North-East-_DOWN_.
}


double Blip::roll() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->attitude[0];
}

double Blip::pitch() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->attitude[1];
}

double Blip::yaw() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->attitude[2];
}


Geometry::Point_2D Blip::rfPolygonCenter() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->rfPolygonCenter;
}

void Blip::setRfPolygonCenter ( const Geometry::Point_2D& center )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->rfPolygonCenter = center;
#ifdef WITH_GRAPHICS
  d->rfPolygonCenterItem->setPos(center.x(), center.y());
#endif // WITH_GRAPHICS
}

Geometry::Polygon_2D Blip::rfPolygon(double const & distanceThreshold ) const
{
  Q_D(const Blip); Q_ASSERT(d);
  const_cast<Blip*>(this)->parent()->updateRfPoly(id(),distanceThreshold);
  return d->rfPolygon;
}

void Blip::setRfPolygon ( const Geometry::Polygon_2D& polygon )
{
  Q_D(Blip); Q_ASSERT(d);
#ifdef WITH_GRAPHICS
  {
    d->rfPolygonItem->setGeometry(polygon);
    emit rfPolygonChanged();
  }
#else // WITH_GRAPHICS
  {
    setRfPolygon(polygon.outer());
  }
#endif // WITH_GRAPHICS
}

void Blip::setRfPolygon ( const Geometry::Polygon_2D::ring_type& ring )
{
#ifdef WITH_GRAPHICS
  {
    Geometry::Polygon_2D tempPoly;
    boost::geometry::assign(tempPoly.outer(),ring);
    setRfPolygon(tempPoly);
  }
#else //WITH_GRAPHICS
  {
    Q_D(Blip); Q_ASSERT(d);
    boost::geometry::assign(d->rfPolygon.outer(),ring); //NOTE: don't use Blip::rfPolygon() as that returns a copy.
    emit rfPolygonChanged();
  }
#endif //WITH_GRAPHICS
}



Geometry::Polygon_2D Blip::lastWpRfPolygon() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->lastWpRfPolygon;
}

void Blip::setLastWpRfPolygon(const Geometry::Polygon_2D& polygon)
{
  Q_D(Blip); Q_ASSERT(d);
#ifdef WITH_GRAPHICS
  {
    d->lastWpRfPolygonItem->setGeometry(polygon);
//     emit lastWpRfPolygonChanged(); //NOTE: this signal currently doesn't exist.
  }
#else // WITH_GRAPHICS
  {
    setLastWpRfPolygon(polygon.outer());
  }
#endif // WITH_GRAPHICS
}


void Blip::setLastWpRfPolygon(const Geometry::Polygon_2D::ring_type& ring)
{
#ifdef WITH_GRAPHICS
  {
    Geometry::Polygon_2D tempPoly;
    boost::geometry::assign(tempPoly.outer(),ring);
    setLastWpRfPolygon(tempPoly);
  }
#else // WITH_GRAPHICS
  {
    Q_D(Blip); Q_ASSERT(d);
    boost::geometry::assign(d->lastWpRfPolygon.outer(),ring); //NOTE: don't use Blip::lastWpRfPolygon() as that returns a copy.
//     emit lastWpRfPolygonChanged(); //NOTE: this signal currently doesn't exist.
  }
#endif // WITH_GRAPHICS
}







double Blip::horizontalFov() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->horizontalFov_deg;
}

void Blip::setHorizontalFov ( const double& hFov )
{
  Q_D(Blip); Q_ASSERT(d);
  d->horizontalFov_deg = hFov;
}


double Blip::routeAltitude() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->routeGraphAltitude;
}


void Blip::setRouteAltiude ( const double& altitude )
{
  Q_D(Blip); Q_ASSERT(d);
  d->routeGraphAltitude = altitude;
}


ElementId Blip::targetId() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->targetElementId;  
}

void Blip::setTargetId(const ElementId& id)
{
  Q_D( Blip ); Q_ASSERT(d);
  if( id != d->targetElementId )
  {
   d->targetElementId = id;
   
   qDebug() << parent()->qualifiedName() << ": Blip" << name() 
   << "set TargetId to" << id;
   
   emit targetChanged(id);
   
#ifdef WITH_GRAPHICS
    {//HACK: MVC update hack
      // MVC: tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
        {// "targetElement" is child #3
          auto top    =  mvc_model->index(3,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
#endif // WITH_GRAPHICS   
   
  }
}

ElementId Blip::closestObstacleId() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->closestObstacleId;  
}

void Blip::setClosestObstacleId(const ElementId& id)
{
  Q_D( Blip ); Q_ASSERT(d);
  if( id != d->closestObstacleId )
  {
    d->closestObstacleId = id;
    emit closestObstacleChanged(id);
   
#ifdef WITH_GRAPHICS
    {//HACK: MVC update hack
      // MVC: tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
        {// "closestObstacle" is child #2
          auto top    =  mvc_model->index(2,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
#endif // WITH_GRAPHICS   
  }
}



Geometry::Polygon_2D Blip::fovPolygon(double const & distanceThreshold ) const
{
  Q_D(const Blip); Q_ASSERT(d);
  const_cast<Blip*>(this)->parent()->updateFovPoly(id(),distanceThreshold);
  return d->fovPolygon;
}

void Blip::setFovPolygon ( const Geometry::Polygon_2D& polygon )
{ 
  Q_D(Blip); Q_ASSERT(d);
#ifdef WITH_GRAPHICS
  d->fovPolygonItem->setGeometry(polygon);
#else // WITH_GRAPHICS
  setFovPolygon(polygon.outer());
#endif // WITH_GRAPHICS
}

void Blip::setFovPolygon ( const Geometry::Polygon_2D::ring_type& ring )
{
#ifdef WITH_GRAPHICS
  {
    Geometry::Polygon_2D tempPoly;
    boost::geometry::assign(tempPoly.outer(),ring);
    setFovPolygon(tempPoly);
  }
#else //WITH_GRAPHICS
  boost::geometry::assign(fovPolygon().outer(),ring);
#endif //WITH_GRAPHICS
}


Geometry::Point_2D Blip::fovPolygonCenter() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->fovPolygonCenter;
}

void Blip::setFovPolygonCenter ( const Geometry::Point_2D& center )
{
  Q_D(Blip); Q_ASSERT(d);
  d->fovPolygonCenter = center;
}

double Blip::fovPolygonHeading() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->fovPolygonHeading;
}

void Blip::setFovPolygonHeading ( const double& heading )
{
  Q_D(Blip); Q_ASSERT(d);
  d->fovPolygonHeading = heading;
}

Geometry::Point_2D Blip::visPolygonCenter() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->visPolygonCenter;
}

void Blip::setVisPolygonCenter ( const Geometry::Point_2D& center )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->visPolygonCenter = center;
}

Geometry::Polygon_2D Blip::visPolygon() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->visPolygon;
}

void Blip::setVisPolygon ( const Geometry::Polygon_2D& polygon )
{ setVisPolygon(polygon.outer()); }

void Blip::setVisPolygon ( const Geometry::Polygon_2D::ring_type& ring )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip);Q_ASSERT(d);
  boost::geometry::assign(d->visPolygon.outer(),ring);
}

ComGraph::Vertex_Descriptor_t Blip::comGraphVertex() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->comGraphVertex;
}

void Blip::setComGraphVertex ( const ComGraph::Vertex_Descriptor_t& descriptor )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->comGraphVertex = descriptor;
}

bool Blip::isConnectedToHost() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->isConnectedToHost;
}

void Blip::setConnectednessToHost ( const bool& connected )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  bool oldStatus = d->isConnectedToHost;
  bool newStatus = connected;
  
  if( oldStatus != newStatus )
  {   
    d->isConnectedToHost = newStatus; // store the new status in the blip
     
    if( newStatus == false )
    { // the blip just got disconnected
      QSet<IDL::Data::StateDescriptorId> mode;
      mode.insert(IDL::Data::STATE_ID_D_NO_LINK);
      setMode(mode);   
      d->currentFlightplanIsValid = false;
      qDebug() << parent()->qualifiedName() << ":" << objectName() << " just disconnected";
    }
    else
    { // the blip just got connected, reflect that in the mode status:
      QSet<IDL::Data::StateDescriptorId> m = mode();
      m.remove(IDL::Data::STATE_ID_D_NO_LINK);
      m.insert(IDL::Data::STATE_ID_D_LINK_UP);
      setMode(m);
      qDebug() << parent()->qualifiedName() << ":" << objectName() << " just connected";
    }
    
#ifdef WITH_GRAPHICS      
    d->updateCurrentFlightplanGraphics();
    d->updateCurrentWaypointGraphics();

    {//HACK: MVC update hack
      // MVC: tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
        {// "comConnectedToHost" is child #0
          auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
#endif // WITH_GRAPHICS
    
//     qDebug() << parent()->qualifiedName() << ": Connectedness to Host of" << name() << "changed to" << newStatus;
    emit connectednessToHostChanged(connected); 
  }
}


bool Blip::isRfRelay() const
{
  Q_D( const Blip); Q_ASSERT(d);
  return d->isRfRelay;
}

void Blip::setRfRelayFlag(const bool& rfRelayFlag)
{
  Q_D(Blip); Q_ASSERT(d);
  bool oldStatus = d->isRfRelay;
  bool newStatus = rfRelayFlag;
  
  if( oldStatus != newStatus )
  { 
    d->isRfRelay = newStatus;
    emit rfRelayFlagChanged(newStatus);
//     qDebug() << parent()->qualifiedName() << ": R/F relay flag of" << name() << "changed to" << newStatus;
    
#ifdef WITH_GRAPHICS
    {// update graphical representation of the R/F relay flag status
      d->updatePalette(); //FIXME: calling this here unconditionally is certainly overkill... 
    }
    {//HACK: MVC update hack
      // MVC: tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
        {// "rfRelayFlag" is child #1
          auto top    =  mvc_model->index(1,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
          auto bottom =  top;
          emit mvc_model->dataChanged(top,bottom);
        }
      }
    }
#endif // WITH_GRAPHICS
  }
}


bool Blip::isJoystickControlled() const
{
  return mode().contains(IDL::Data::STATE_ID_V_JOYSTICK);
}












int Blip::closestObstacle() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->closestObstacle_index;
}

void Blip::setClosestObstacle(const int& obstacleIndex)
{
  Q_D(Blip); Q_ASSERT(d);
  
  if( obstacleIndex != d->closestObstacle_index )
  {
   d->closestObstacle_index = obstacleIndex;
   emit closestObstacleChanged(obstacleIndex);
   
  }
  
}


IDL::Data::Flightplan Blip::flightplan() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->currentFlightplan;
}



bool Blip::setFlightplan ( const IDL::Data::Flightplan& idlPlan )
{
  Q_D(Blip); Q_ASSERT(d);
  
//   qDebug() << parent()->qualifiedName() << ": Setting Flightplan for"
//     << name() << "(Length:" << idlPlan.waypoints.length() 
//     << ", Hash:" << IDL::toQByteArray(idlPlan.hash).toHex() <<")";
  
  d->currentFlightplanIsValid = true;
  
  if(idlPlan.hash != d->currentFlightplan.hash )
  {
    d->currentFlightplan = idlPlan;
    d->currentWaypointIndex = 0;
    
#ifdef WITH_GRAPHICS
    {
      d->updateCurrentFlightplanGraphics();
      d->updateCurrentWaypointGraphics();
    }
#endif // WITH_GRAPHICS
    
//     qDebug() << parent()->qualifiedName() << ": Flightplan differes from stored one.";
    
    emit flightplanChanged();
    return true;
  }
  else
  { return false; }
}

Waypoint Blip::currentWaypoint() const
{
  Q_D(const Blip); Q_ASSERT(d);
  auto idlWp = d->currentFlightplan.waypoints[d->currentWaypointIndex];
  return Waypoint(idlWp);
}

unsigned int Blip::currentWaypointIndex() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->currentWaypointIndex;
}


bool Blip::setCurrentWaypointIndex ( const unsigned int& index )
{
  Q_D(Blip); Q_ASSERT(d);
  if( d->currentWaypointIndex != index )
  {
    d->currentWaypointIndex = index;
#ifdef WITH_GRAPHICS
    d->updateCurrentFlightplanGraphics();
    d->updateCurrentWaypointGraphics();
#endif // WITH_GRAPHICS
    return true;
  }
  else
  { return false; }
}


IDL::Data::ModeDescriptor Blip::modeDescriptor() const
{
  Q_D(const Blip); Q_ASSERT(d);
    
  IDL::Data::ModeDescriptor md(d->mode.size()); //NOTE: The constructor only sets the maximum...
  md.length(d->mode.size()); //NOTE: ...calling length(DDS_Long) actually allocates the memory
 
  int i = 0;
  for( IDL::Data::StateDescriptorId state : d->mode )
  { md[i++]=state; }
  
  return md;
//   return d->modeDescriptor;
}


void Blip::setModeDescriptor ( const IDL::Data::ModeDescriptor& modeDescriptor )
{
 setMode(modeDescriptor); 
}

QSet<IDL::Data::StateDescriptorId> Blip::mode() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->mode;
}

void Blip::setMode ( const IDL::Data::ModeDescriptor& modeDescriptor )
{
  QSet<IDL::Data::StateDescriptorId> mode;
  for( int i = 0; i<modeDescriptor.length(); ++i)
  { mode << modeDescriptor[i]; }
  
  setMode( mode ); 
}
  
void Blip::setMode ( const QSet< IDL::Data::StateDescriptorId >& mode )
{
  Q_D(Blip); Q_ASSERT(d);
  
  if( d->mode != mode )
  {
    d->mode = mode;
//     qDebug() << name() << ": mode change: from" << d->mode << "to" << mode;
    emit modeChanged();
    
#ifdef WITH_GRAPHICS
    {// update graphical representation of the mode of the Blip
      d->updatePalette(); //FIXME: calling this here unconditionally is certainly overkill... 
      bool isJoystickControlled = mode.contains(IDL::Data::STATE_ID_V_JOYSTICK);
      setFovPolygonVisible( isJoystickControlled ); 
      setBodyFrameVisible( isJoystickControlled );
    }
    
    {//HACK: MVC update hack
      // tell associated models that they need to update the views
      QAbstractItemModel* mvc_model =  model();
      if( mvc_model )
      {
        {// modeDescriptor is the "value" of the Blip(-Item)
          auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),index().parent());//FIXME: Magic Numbers: row
          auto bottom =  top;
          emit model()->dataChanged(top,bottom);
        }
      }
    }
#endif // WITH_GRAPHICS
  }
}

#ifdef WITH_GRAPHICS 

void Blip::setMainColor ( const QColor color )
{
  Q_D(Blip); Q_ASSERT(d);
  d->mainColor = color;
  
  using namespace Graphics_2D;
  
  d->rfPolygonCenterItem->setPen(Pens::Outline(color));
  d->rfPolygonCenterItem->setBrush(Brushes::FloodFill(color));
  
  d->lastWpRfPolygonItem->setPen(Pens::Outline(color));
  d->lastWpRfPolygonItem->setBrush(Qt::NoBrush);
}

QColor Blip::mainColor() const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->mainColor;
}


void Blip::setDatumItem ( QGraphicsItem*const datumItem )
{
  // QMutexLocker locker(&mutex);
  Q_D(Blip); Q_ASSERT(d);
  d->datumItem = datumItem;
  d->rfPolygonItem->setParentItem( datumItem );
  d->rfPolygonItem->setZValue(NodePrivate::BlipRfPolygonZLevel);
  d->lastWpRfPolygonItem->setParentItem( datumItem );
  d->lastWpRfPolygonItem->setZValue(NodePrivate::BlipRfPolygonZLevel);
  d->fovPolygonItem->setParentItem( datumItem );
  d->fovPolygonItem->setZValue(NodePrivate::BlipFovPolygonZLevel);
  d->rfPolygonCenterItem->setParentItem( datumItem );
  d->rfPolygonCenterItem->setZValue(NodePrivate::BlipRfPolygonCenterZLevel);
  d->bodyCarriedFrameItem->setParentItem( datumItem );
  d->bodyCarriedFrameItem->setZValue(NodePrivate::BlipBodyCarriedFrameZLevel);
 
//   d->velocityItem->setParentItem( d->bodyCarriedFrameItem ); 
//   d->bodyFrameItem->setParentItem( d->bodyCarriedFrameItem );
}


Graphics_2D::Polygon* Blip::rfPolygonItem()
{
  Q_D(Blip); Q_ASSERT(d);
  return d->rfPolygonItem;
}

Graphics_2D::Polygon* Blip::lastWpRfPolygonItem()
{
  Q_D(Blip); Q_ASSERT(d);
  return d->lastWpRfPolygonItem;
}

void Blip::setPalette(const Blip::Palette* palette)
{
  Q_D(Blip); Q_ASSERT(d);
  d->palette = palette;
  d->updatePalette();
}

Graphics_2D::Polygon* Blip::fovPolygonItem()
{
  Q_D(Blip); Q_ASSERT(d);
  return d->fovPolygonItem;
}

/** \internal TreeItem */
QVariant Blip::data ( int column, int role) const
{
  Q_D(const Blip); Q_ASSERT(d);
  return d->modelData(column,role);
}

bool Blip::isFovPolygonVisible() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->fovPolygonItem->isVisible(); 
}


void Blip::setFovPolygonVisible ( bool visible )
{
  Q_D(Blip); Q_ASSERT(d);
  
  if( visible == true and d->horizontalFov_deg <= 0 )
  { return; } // there wouldn't be anything there to show anyways...
  
  if( isFovPolygonVisible() != visible )
  {
    d->fovPolygonItem->setVisible(visible);
//     d->showFovItem = visible;
    emit fovPolygonVisibilityChanged(visible);
  }

//   //TODO: the stuff below shouldn't be necessary as MVC could listen to the 
//   // fovPolygonVisibilityChanged signal.
//   
//   {//HACK: MVC update hack
//     // MVC: tell associated models that they need to update the views
//     QAbstractItemModel* mvc_model =  model();
//     if( mvc_model )
//     {
//       auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
//       {// "fovPolyVisible" is child #0
//         auto top    =  mvc_model->index(0,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
//         auto bottom =  top;
//         emit mvc_model->dataChanged(top,bottom);
//       }
//     }
//   }
}

bool Blip::isBodyFrameVisible() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->bodyFrameItem->areAxesVisible();
}

void Blip::setBodyFrameVisible(bool visible)
{
  Q_D(Blip); Q_ASSERT(d);
//   if( d->bodyFrameItem->isVisible()!=visible )
//   {
//     d->bodyFrameItem->hide();
  if( isBodyFrameVisible()!=visible )
  {
    d->bodyFrameItem->setAxesVisible(visible);
    emit bodyFrameVisibilityChanged(visible);
  }
}

bool Blip::isRfPolygonVisible() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->rfPolygonItem->isVisible();
}


void Blip::setRfPolygonVisible ( bool visible )
{
  Q_D(Blip); Q_ASSERT(d);
  
  if( isRfPolygonVisible() != visible )
  { 
    d->rfPolygonItem->setVisible(visible);
    d->rfPolygonCenterItem->setVisible(visible);
    emit rfPolygonVisibilityChanged(visible);
  }
  
//   //TODO: the stuff below shouldn't be necessary as MVC could listen to the 
//   // fovPolygonVisibilityChanged signal.
//   
//   {//HACK: MVC update hack
//     // MVC: tell associated models that they need to update the views
//     QAbstractItemModel* mvc_model =  model();
//     if( mvc_model )
//     {
//       auto item = static_cast<TreeItem*>(d->mvSubItems->debug);
//       {// "rfPolyVisible" is child #1
//         auto top    =  mvc_model->index(1,static_cast<int>(DataAspect::value),item->index());//FIXME: Magic Numbers: row
//         auto bottom =  top;
//         emit mvc_model->dataChanged(top,bottom);
//       }
//     }
//   }
}


bool Blip::isLastWpRfPolygonVisible() const
{
  Q_D( const Blip ); Q_ASSERT(d);
  return d->lastWpRfPolygonItem->isVisible();
}


void Blip::setLastWpRfPolygonVisible ( bool visible )
{
  Q_D(Blip); Q_ASSERT(d);
  
  if( isLastWpRfPolygonVisible() != visible )
  { 
    d->lastWpRfPolygonItem->setVisible(visible);
//     emit lastWpRfPolygonVisibilityChanged(visible);
  }
}



void Blip::setParameter(const Blip::GraphicsParameter& parameter)
{
  Q_D(Blip); Q_ASSERT(d);
  d->flightPlanGraphicsFutureWpCount = parameter.futureWpCount;
  d->flightPlanGraphicsPastWpCount   = parameter.pastWpCount;
  d->updateCurrentFlightplanGraphics();
}


QAction* Blip::menuAction()
{
  Q_D( Blip ); Q_ASSERT(d);
  
  d->updatePersistentContextMenu();
  return d->persistentContextMenu.menuAction();
}



#endif // WITH_GRAPHICS

#include "blip.moc"
