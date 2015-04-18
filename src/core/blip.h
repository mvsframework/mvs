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

#ifndef BLIP_H
#define BLIP_H

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>
#include <QtCore/QMutex>
#include <QtCore/QSet>

#include <amg.hpp>

#include "config.h"

#include "nodeid.h"
#include "IDL/data.h"
#include "Geometry/geometry.h"
#include "Graph/comgraph.h"
#include "GNC/flightplan.h"
#include "Environment/element.h"


#ifdef WITH_GRAPHICS
  #include <QtWidgets/QAction>
  #include <QtWidgets/QAbstractGraphicsShapeItem>
  
  #include "Graphics/geometryitem.h"
  #include "Graphics/palette.h"
  
  #include "MVC/treeitem.h"
#endif // WITH_GRAPHICS

class QGraphicsScene;

class Node; // forward declaration
class BlipPrivate; // forward declaration

/** \brief The records one Nodes keeps about another Node.
 * As Nodes partake in the network's communcation, they build up a local 
 * database of information they keep about the other Node members of the 
 * network. A Blip holds the information collected for and about a particular
 * Node.
 * 
 * Types of data within a Blip:
 * 
 * - <b>Primary Data:</b> independet data that is set from the outside, e.g.
 *   position3D. There are public accessors and mutators for this data.
 * 
 * - <b>Secondary Data:</b> data that can be solemnly computed from the primary 
 *   data, i.e. no further Blip-external data is necessary. An example would be
 *   position2D. There are only public accessors for this data as it is 
 *   immutuable from the outside.
 * 
 * - <b>Tertiary Data:</b> this is blip specific data that is simply stored in 
 *   the Blip as that is a natural place for it. Tertiary data is data that 
 *   depends on primary and/or secondary data in combination with further 
 *   external data. An example would be the rfPolygon, as that depends on a 
 *   Blip's position2D an a Node's environment2D. There are public accessors as 
 *   well as mutators for tertiary data.
 * 
 * \internal When dealing with the graphics realted to representing Blips, keep 
 * in mind that "[QWidgets]  and all of its subclasses, along with other 
 * GUI-related classes (even not QObject-based, like QPixmap) are not reentrant 
 * either: they can be used exclusively from the GUI thread."
 * (from https://qt-project.org/wiki/Threads_Events_QObjects)
 */
#ifdef WITH_GRAPHICS
class Blip : public QObject, public TreeItem {  
#else // WITH_GRAHICS
class Blip : public QObject {
#endif // WITH_GRAPHICS
Q_OBJECT

Q_PROPERTY(NodeId id READ id WRITE setId)
// Q_PROPERTY(IDL::Data::NodeType id READ type WRITE setType)
Q_PROPERTY(QString name READ name WRITE setName)
Q_PROPERTY(bool connectednessToHost READ isConnectedToHost WRITE setConnectednessToHost NOTIFY connectednessToHostChanged)
Q_PROPERTY(bool rfRelayFlag READ isRfRelay WRITE setRfRelayFlag NOTIFY rfRelayFlagChanged)
Q_PROPERTY(bool joystickControlled READ isJoystickControlled)
Q_PROPERTY(ElementId closestObstacle READ closestObstacleId WRITE setClosestObstacleId NOTIFY closestObstacleChanged)
Q_PROPERTY(ElementId target READ targetId WRITE setTargetId NOTIFY targetChanged)


#ifdef WITH_GRAPHICS
Q_PROPERTY(bool bodyFrameVisibility  READ isBodyFrameVisible  WRITE setBodyFrameVisible  NOTIFY bodyFrameVisibilityChanged)
Q_PROPERTY(bool rfPolygonVisibility  READ isRfPolygonVisible  WRITE setRfPolygonVisible  NOTIFY rfPolygonVisibilityChanged)
Q_PROPERTY(bool fovPolygonVisibility READ isFovPolygonVisible WRITE setFovPolygonVisible NOTIFY fovPolygonVisibilityChanged)
// Q_PROPERTY(bool visPolygonVisibility READ isVisPolygonVisible WRITE setVisPolygonVisible NOTIFY visPolygonVisibilityChanged)
#endif // WITH_GRAPHICS


Q_SIGNALS:
  /** \brief Indicates a change in the connectedness state of the Blip. */
  void connectednessToHostChanged(bool /*isConnected*/);
  
  /** \brief Indicates a change of the R/F relay flag of the Blip. */
  void rfRelayFlagChanged(bool /*isRfRelay*/);
  
  /** \brief Indicates a change in the position of the Blip. */
  void positionChanged();
  
  /** \brief Indicates a change in the mode of the Blip. */
  void modeChanged();
  
  /** \brief Indicates a change in the R/F polygon geometry of the Blip. */
  void rfPolygonChanged();
  
  /** \brief Indicates a change of the closest obstacle to the Blip. */
  void closestObstacleChanged(int /*obstacleIndex*/);
  void closestObstacleChanged(ElementId /*id*/);
  
  /** \brief Indicates a change of the target Element of the Blip. */
  void targetChanged(ElementId /*id*/);
  
  /** \brief Indicates a change in the flightplan of the Blip. */
  void flightplanChanged();
  
#ifdef WITH_GRAPHICS
  /** \brief Indicates a change in the visibility of the body frame item. */
  void bodyFrameVisibilityChanged(bool /*visibility*/); 
  /** \brief Indicates a change in the visibility of the R/F polygon item. */
  void rfPolygonVisibilityChanged(bool /*visibility*/); 
  /** \brief Indicates a change in the visibility of the FOV polygon item. */
  void fovPolygonVisibilityChanged(bool /*visibility*/); 
//   /** \brief Indicates a change in the visibility of the visibility polygon item. */
//   void visPolygonVisibilityChanged(bool /*visibility*/); 
  /** \brief Indicates that a context Menu for this blip has been requested at the given position. */
  void contextMenuRequested(QPoint /*position*/);
#endif // WITH_GRAPHICS
  
  
public:
  /** \brief Constructor. */
  Blip(Node* parent = nullptr);

  /** \brief Destructor.
   * \internal Due to the use of QScopedPointers for PIMPL the destructor seems
   * to need an implementation in the .cpp file <b>even if it is empty</b>.
   */
  virtual ~Blip();
  
    
  /** \brief Get the parent Node of this Blip. */
  Node* parent();
  
  //
  // The basic (static) primary data
  //
  
  /** \brief The (assigned) name of the Blip. 
   * \deprecated {Use QObject::objectName() instead.}
   *
   * The initial value is the name as detected during Node discovery. 
   * \sa setName()
   */
  QString                name() const; 
  
  /** \brief Set the name that should be used for this Blip.
   * \deprecated {Use setObjectName() instead.}
   *
   * \sa name()
   */
  void                setName(QString const & name);
    

  /** \brief The UUID of the node this Blip is representing. */
  NodeId                 id() const;  
  void                setId(NodeId const & id);
  
  /** \brief The type of the node this Blip is representing. */
  IDL::Data::NodeType    type() const; 
  void                setType(IDL::Data::NodeType const & type);
  
  
  //
  // Primary Data (updated via IDL::Messages)
  //
    
  /** \brief A comprehensive description of the  state of the Blip.
   * \deprecated
   * This returns a description of all states the blip's state machine is/was 
   * in.
   */
  IDL::Data::ModeDescriptor modeDescriptor() const;
  
  /** \brief Set the data describing the (current) mode.
   * \deprecated
   */
  virtual void setModeDescriptor(IDL::Data::ModeDescriptor const & modeDescriptor);
  
  /** \brief A comprehensive description of the mode state of the Blip.
   * This returns a description of all states the blip's state machine is/was 
   * in.
   */
  QSet<IDL::Data::StateDescriptorId> mode() const;
  
  /** \brief Set the data describing the (current) mode. */
  virtual void setMode( const QSet< IDL::Data::StateDescriptorId >& mode);
  /** \brief Set the data describing the (current) mode.
   * \overload
   */
  virtual void setMode( IDL::Data::ModeDescriptor const & modeDescriptor );
  
  AMG::Vector            position3D() const;
  virtual void        setPosition3D(AMG::Vector const& position);
  
  AMG::Vector            velocity() const;
  virtual void        setVelocity(AMG::Vector const& velocity);
  
  AMG::EulerAngleTupel   attitude() const;
  virtual void        setAttitude(AMG::EulerAngleTupel const& eulerAngles);

  IDL::Data::Flightplan flightplan() const;  
  /** \brief Store the Blip's Flightplan.
   * \return TRUE if the plan is different from the previously stored one; 
   * FALSE otherwise. */
  virtual bool setFlightplan(IDL::Data::Flightplan const & idlPlan);
  
  unsigned int currentWaypointIndex() const;
  /** \brief Store the Blip's current Flightplan Waypoint index.
   * \return TRUE if the index is different from the previously stored one; 
   * FALSE otherwise. */
  virtual bool setCurrentWaypointIndex(unsigned int const & index);
  
   
  ComGraph::Vertex_Descriptor_t comGraphVertex() const;
  virtual void               setComGraphVertex(ComGraph::Vertex_Descriptor_t const & descriptor);

  
  /** \brief Get the horizontal Field-of-View in degree.
   * \return The horizontal Field-of-View in degree.
   */  
  double          horizontalFov() const;
  
  /** \brief Set the horizontal Field-of-View in degree. */
  virtual void setHorizontalFov(double const& hFov); 
  
  /** \brief Get the (datum) altitude of the route graph in meter. */
  double routeAltitude() const;
  
  /** \brief Set the (datum) altitude of the route graph in meter. */
  virtual void setRouteAltiude( const double& altitude );
  
  /** \brief Get the ID of the Element currently targeted. */
  ElementId targetId() const; 
  /** \brief Set the ID associated to the Element targeted. */ 
  virtual void setTargetId(ElementId const& id);
  
  /** \brief Get the ID of the Element currently being the closest obstacle. */
  ElementId closestObstacleId() const;
  
  /** \brief Set the ID of the Element currently being the closest obstacle. */
  virtual void setClosestObstacleId(ElementId const& id);
  
  //
  // Secondary Data (computable from primary data)
  //
  
  /** \brief Get the (datum) altitude in meter. */
  double altitude() const;
  
  /** \brief Get the roll angle phi in degrees. */
  double roll() const;
  /** \brief Get the pitch angle theta in degrees. */
  double pitch() const;
  /** \brief Get the yaw angle psi in degrees. */
  double yaw() const;
  
  /** \brief Get the 2D position wrt. the datum in meter. */
  Geometry::Point_2D    position2D() const;
//   virtual void       setPosition2D(Geometry::Point_2D const& position);
  
    
  /** \brief Get the current waypoint this Blip is targeting. 
   * 
   * The \e current waypoint is the waypoint a vehicle is currently trying to 
   * reach.
   */
  Waypoint currentWaypoint() const;
  
  
  //
  // Tertiary Data (depending on internal and external data)
  //
  
  /** \brief Get a copy of the R/F polygon of the Blip.
   * \details Calls Node::updateRfPoly to ensure that the R/F polygon is 
   * updated if the Blip should have moved more than the indicated threshold.
   * \param[in] distanceThreshold The update threshold level in [m]. */
  Geometry::Polygon_2D  rfPolygon( double const & distanceThreshold = 2.0 ) const;
  virtual void       setRfPolygon(Geometry::Polygon_2D const& polygon);
  virtual void       setRfPolygon(Geometry::Polygon_2D::ring_type const& ring);
  
  /** \brief Get a copy of the R/F polygon center of the Blip. */  
  Geometry::Point_2D  rfPolygonCenter() const;
  virtual void     setRfPolygonCenter(Geometry::Point_2D const& center);

    /** \brief Get a copy of the R/F polygon of the Blip at its last Waypont. */
  Geometry::Polygon_2D  lastWpRfPolygon() const;
  virtual void       setLastWpRfPolygon(Geometry::Polygon_2D const& polygon);
  virtual void       setLastWpRfPolygon(Geometry::Polygon_2D::ring_type const& ring);
  
  
  /** \brief Get a copy of the visbility polygon of the Blip. */
  Geometry::Polygon_2D  visPolygon() const;
  virtual void       setVisPolygon(Geometry::Polygon_2D const& polygon);
  virtual void       setVisPolygon(Geometry::Polygon_2D::ring_type const& ring);
    
  /** \brief Get a copy of the visibility polygon center of the Blip. */
  Geometry::Point_2D  visPolygonCenter() const;
  virtual void     setVisPolygonCenter(Geometry::Point_2D const& center);

  /** \brief Get a copy of the field-of-view polygon of the Blip.
   * \details Calls Node::updateFovPoly to ensure that the FOV polygon is 
   * updated if the Blip should have moved more than the indicated threshold.
   * \param[in] distanceThreshold The update threshold level in [m]. */
  Geometry::Polygon_2D  fovPolygon( double const & distanceThreshold = 2.0 ) const;
  virtual void       setFovPolygon(Geometry::Polygon_2D const& polygon);
  virtual void       setFovPolygon(Geometry::Polygon_2D::ring_type const& ring);
  
  /** \brief Get a copy of the field-of-view polygon center of the Blip. */
  Geometry::Point_2D fovPolygonCenter() const;
  virtual void    setFovPolygonCenter(Geometry::Point_2D const& center);
  
  double             fovPolygonHeading() const;
  virtual void    setFovPolygonHeading(double const & heading);  
  
  /** \brief The (COM Graph) connectedness of this Blip to the hosting Node.
   * \return TRUE if the Blip has a working COM connection to the hosting node, 
   * i.e. the Blip is a member of the connected component of the hosting Node; 
   * FALSE otherwise.
   */
  bool           isConnectedToHost() const;
  virtual void  setConnectednessToHost( bool const & connected);

//   /** \brief The (COM Graph) connectedness of this Blip to the commanding 
//    * ControlStation.
//    * \fixme This is technically only needed for hosts of the Vehicle type. 
//    * \return TRUE if the Blip has a working COM connection to the commanding 
//    * control station node \e without the hosting node, 
//    * i.e. the Blip is a member of the connected component of the commanding 
//    * ControlSation when the hosting Node is not part of the COM graph; 
//    * FALSE otherwise.
//    */
//   bool           isConnectedToCommandingControlStation() const;
//   virtual void  setConnectednessToCommandingControlStation( bool const & connectedness);
  
  
  /** \brief The status of this Blip as being a R/F relay node to some Node of
   * interest (i.e. not the host) in the network.
   * \fixme This is technically not needed for pure Node hosts, but Vehicle and
   * ControlSation need it. 
   * \return TRUE if the Blip has a working COM connection to (or is part of 
   * chain to) the other Node of interest \e without the hosting node being 
   * part of the network, i.e. the Blip is a member of the connected component 
   * of the other Node of interest when the hosting Node is not part of the COM 
   * graph; FALSE otherwise. */
  bool          isRfRelay() const;
  virtual void setRfRelayFlag( bool const & rfRelayFlag);

  /** \brief The joystick control status of the Blip. 
   * \return TRUE if the Blip is under joystick control; FALSE otherwise.
   */
  bool isJoystickControlled() const;
  
  
  /** \deprecated Use closestObstacleId(). */
  int             closestObstacle() const;
  /** \deprecated Use setClosestObstacleId(). */
  virtual void setClosestObstacle( int const & obstacleIndex);
 
#ifdef WITH_GRAPHICS
  
  /** \brief The palette of colors and styles used to draw a Blip. */
  struct Palette {
    Graphics_2D::Palette rfPoly_relay;
    Graphics_2D::Palette rfPoly_nonRelay;
    Graphics_2D::Palette rfPoly_ownship;
    
    Graphics_2D::Utensils fovPoly_joystick;
    Graphics_2D::Utensils fovPoly_nonJoystick;
  };
  
  
  /** \brief Set the palette used to draw the rfPolygonItem.
   * \note The Blip does not take ownership of the pointer.
   */
  virtual void setPalette(Palette const* palette);
//   Palette const* palette() const;
  
//   /** \brief Set the palette used to dras the R/F polygon when the Blip acts as a relay. */
//   virtual void         setRfRelayPalette(Graphics_2D::Palette const * palette);
  
//   /** \brief Set the palette used to dras the R/F polygon when the Blip does not act as a relay. */
//   virtual void         setRfNonRelayPalette(Graphics_2D::Palette const * palette);
  

  QColor  mainColor() const;
  void setMainColor(QColor const color);
  
  
  /** \brief Set the graphical item representing the Datum of the hosting Node.
   * 
   * As all Blip data is stored relative to the Datum of the hosting
   * Node, all geometric information is relative to the corresponding frame of
   * reference. This function tells the Blip which QGraphicsItem--or to be more
   * precise, which QGraphicsItem's internal coordinate system--represents that
   * frame.
   * 
   * Internally, this function calls QGraphicsItem::setParentItem() on all 
   * graphical elements, which, on the first call, effectively adds them to the
   * scene (assuming the datumItem is already a member of the scene). However,
   * not all elements might be set to "visible".
   */
  virtual void setDatumItem(QGraphicsItem* const datumItem);
  
  //@{
  /** \brief Set the visibility of the representation of the body frame. */
  bool           isBodyFrameVisible() const;
  virtual void  setBodyFrameVisible(bool visible);
  virtual void showBodyFrame(){ setBodyFrameVisible(true); };
  virtual void hideBodyFrame(){ setBodyFrameVisible(false); };
  //@}
  
  Graphics_2D::Polygon*   rfPolygonItem();  
  
  //@{
  bool            isRfPolygonVisible() const;
  virtual void   setRfPolygonVisible(bool visible);
  virtual void  showRfPolygon(){ setRfPolygonVisible(true);  };
  virtual void  hideRfPolygon(){ setRfPolygonVisible(false); };
  //@}
  
  
  Graphics_2D::Polygon* lastWpRfPolygonItem();
  
  //@{
  bool            isLastWpRfPolygonVisible() const;
  virtual void   setLastWpRfPolygonVisible(bool visible);
  virtual void  showLastWpRfPolygon(){ setLastWpRfPolygonVisible(true);  };
  virtual void  hideLastWpRfPolygon(){ setLastWpRfPolygonVisible(false); };
  //@}
  
  
  
  Graphics_2D::Polygon*  fovPolygonItem();

  //@{
  bool            isFovPolygonVisible() const;
  virtual void   setFovPolygonVisible(bool visible);
  virtual void  showFovPolygon(){ setFovPolygonVisible(true);  };
  virtual void  hideFovPolygon(){ setFovPolygonVisible(false); };
  //@}
  
  /** \brief The parameters affecting the graphics for the blip. */
  struct GraphicsParameter {
    /** \brief The number of future (next) waypoints drawn in flightplan graphcis.
     * 
     * The number of future (next) waypoints, i.e. after the current waypoint, 
     * that should be drawn when plotting the flightplan.
     * Counts less than 0 select all Waypoints.
     */
    int futureWpCount = -1;
    
    /** \brief The number of past (passed) waypoints drawn in flightplan graphcis.
     * 
     * The number of past (passed) waypoints,  i.e. before the current waypoint, 
     * that should be drawn when plotting the flightplan.
     * Counts less than 0 select all Waypoints.
     */
    int pastWpCount = -1;
    
    
    /** \brief The main color used to identify this blip.
     * \sa Qt::GlobalColor
     * https://qt-project.org/doc/qt-5.0/qtcore/qt.html#GlobalColor-enum
     */
//     QColor mainColor = Qt::gray;
  };
  
  /** \brief Set and apply a set of parameter. */
  virtual void setParameter(GraphicsParameter const & parameter);
  
  /** \brief Get a context menu action for the blip. */
  QAction* menuAction();
    
  //
  // TreeItem functions
  // 
  
  virtual QVariant data ( int column, int role = Qt::DisplayRole ) const;

  
#endif // WITH_GRAPHICS

  
protected:
  QScopedPointer< BlipPrivate> const d_ptr;
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Blip( BlipPrivate& dd, Node* parent );
  

private:
  Q_DISABLE_COPY(Blip)
  Q_DECLARE_PRIVATE(Blip)
//   mutable QMutex mutex;
};

#endif // BLIP_H
