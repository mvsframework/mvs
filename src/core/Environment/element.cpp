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

#include "element.h"
#include "element_p.h"

#include <assert.h>

#include "Nodes/node_p.h"

#ifdef WITH_GRAPHICS
  #include "Graphics/palette.h"
  #include <QtWidgets/QGraphicsSceneEvent>
#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  ElementPrivate                                                            //
//                                                                            //
//############################################################################//

ElementPrivate::ElementPrivate(Element* q)
  :q_ptr(q)
{//NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!
  
#ifdef WITH_GRAPHICS
  {    
    mainColor = Graphics_2D::Colors::randomColor();
    
    shapeItem = new PolygonItem(this,shape);
    convexHullItem = new PolygonItem(this,convexHull);
    convexHullItem->setVisible(false); //FIXME: Magic Numbers: default visibility for the convex hull item is set here.  
  }
#endif // WITH_GRAPHICS
}

ElementPrivate::~ElementPrivate()
{
}

#ifdef WITH_GRAPHICS
void ElementPrivate::setColor(QColor const& color)
{
  using namespace Graphics_2D;
  
  shapeItem->setPen(Pens::Outline(color));
  shapeItem->setBrush(Brushes::FloodFill(color));
        
  convexHullItem->setPen(Pens::Outline(color));
  convexHullItem->setBrush(Qt::NoBrush);
}

void ElementPrivate::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{  
  Q_Q(Element); Q_ASSERT(q);
  
  QMenu menu;
  
  if( auto node = dynamic_cast<Node*>(q->parent()) )
  {
    node->buildElementContextMenu(&menu,id);
  }
  else
  {
    menu.addAction(q->menuAction()); 
  }
  menu.exec(event->screenPos());
}


void ElementPrivate::updatePersistentContextMenu()
{
  Q_Q(Element); Q_ASSERT(q);
  showConvexHullAction->setChecked(q->isConvexHullVisible());
}

#endif // WITH_GRAPHICS

//############################################################################//
//                                                                            //
//  ElementPrivate::PolygonItem                                               //
//                                                                            //
//############################################################################//
#ifdef WITH_GRAPHICS

#endif // WITH_GRAPHICS
//############################################################################//
//                                                                            //
//  Element                                                                   //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Element level construction needs to happen in 
 * Element(ElementPrivate& dd, ...)!
 */
Element::Element(QObject* parent)
  :Element(*new ElementPrivate( this ), parent)
{}

Element::Element(ElementPrivate& dd, QObject* parent)
  :QObject(parent)
  ,d_ptr(&dd)
{
  Q_D( Element ); Q_ASSERT(d);
  
#ifdef WITH_GRAPHICS
  //
  // Create the Element's persistent context menu actions
  //
  {    
    d->showConvexHullAction = new QAction(this);
    d->showConvexHullAction->setText( tr("&Show Convex Hull"));
    d->showConvexHullAction->setCheckable(true);
    d->showConvexHullAction->setChecked(isConvexHullVisible());
    
    connect(d->showConvexHullAction, &QAction::triggered,
            this, &Element::setConvexHullVisibility);  
    
    d->persistentContextMenu.addAction(d->showConvexHullAction);
  }

  {
  setMainColor(d->mainColor);
  
  d->shapeItem->setToolTip(tr("(unspecified element)"));
  d->shapeItem->setCursor(Qt::ArrowCursor);
  connect(this, &QObject::objectNameChanged,
          [=](QString name) {
            d->shapeItem->setToolTip(name);
            d->persistentContextMenu.setTitle(name);
          });
  }
#endif // WITH_GRAPHICS
}

Element::~Element()
{
}


Element::Element(const Element& other)
  :Element(other.parent())
{
  *this = other;
}



Element& Element::operator=(const Element& other)
{
  // check for self assingment
  if( this == &other )
  { return *this; }
  
  setParent(other.parent());
  setId(other.id());
  setObjectName(other.objectName());
  setShape(other.shape());
  
  return *this;
}

bool Element::operator==(const Element& other)
{
  return id()==other.id();
}


ElementId Element::id() const
{
    Q_D(const Element);
    return d->id;
}

void Element::setId(ElementId const& id)
{
  Q_D( Element ); Q_ASSERT(d);
  if( d->id != id )
  {
   d->id = id;
   emit idChanged(id);
  }
}

Geometry::Polygon_2D& Element::mutableShape()
{
  Q_D(Element); Q_ASSERT(d);
  return d->shape;
}


Geometry::Polygon_2D const & Element::shape() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->shape;
}

void Element::setShape(const Geometry::Polygon_2D& shape)
{
  Q_D(Element); Q_ASSERT(d);
  
#ifdef WITH_GRAPHICS
  {
    // set the shape of the obstacle
    Geometry::Polygon_2D tempShape = shape;
    boost::geometry::correct(tempShape);
    d->shapeItem->setGeometry(tempShape); // this assigns the corrected tempShape to d->shape
    
    // compute the convex hull of the shape and set it
    Geometry::Polygon_2D tempConvexHull;
    boost::geometry::convex_hull(tempShape,tempConvexHull); 
    d->convexHullItem->setGeometry(tempConvexHull);
    
    // compute the centroid of the hull and set it
    boost::geometry::centroid(tempShape,d->shapeCentroid);
  }
#else // WITH_GRAPHICS
  {
    boost::geometry::assign(d->shape,shape);
    boost::geometry::correct(d->shape);
    
    // compute the convex hull of the shape and set it
    boost::geometry::convex_hull(d->shape,d->convexHull); 
    
    // compute the centroid of the hull and set it
    boost::geometry::centroid(d->convexHull,d->shapeCentroid);
  }
#endif // WITH_GRAPHICS

  emit shapeChanged();
}

Geometry::Polygon_2D Element::convexHull() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->convexHull;
}

Geometry::Point_2D Element::centroid() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->shapeCentroid;
}

#ifdef WITH_GRAPHICS

void Element::setParentItem(QGraphicsItem* newParent)
{
  Q_D(Element); Q_ASSERT(d);
  
//   qDebug() << objectName() << ": d->shapeItem->parentItem()=" << d->shapeItem->parentItem();
//   qDebug() << objectName() << ": newParent=" << newParent;
//   qDebug() << objectName() << ": newParent->childItems().count()=" << newParent->childItems().count();
  
  d->convexHullItem->setParentItem(newParent);
  d->shapeItem->setParentItem(newParent);
  
//   qDebug() << objectName() << ": newParent->childItems().count()=" << newParent->childItems().count();
//   qDebug() << objectName() << ": d->shapeItem =" << d->shapeItem;
}



QColor Element::mainColor() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->mainColor;
}

void Element::setMainColor(const QColor& color)
{
  Q_D(Element); Q_ASSERT(d);
  
  d->mainColor = color;
  d->setColor(color);
}


void Element::markAsTarget()
{
  Q_D(Element); Q_ASSERT(d);
  d->tags &= ElementPrivate::Tag::HasTargetMarker | ElementPrivate::Tag::Target ;
  d->setColor(Graphics_2D::Colors::targetElement);
}

void Element::markAsNonTarget()
{
  Q_D(Element); Q_ASSERT(d);
  d->tags &= ElementPrivate::Tag::HasTargetMarker | !ElementPrivate::Tag::Target;
  d->setColor(Graphics_2D::Colors::nonTargetElement);
}

void Element::removeTargetMarker()
{
  Q_D(Element); Q_ASSERT(d);
  d->tags &= ! ElementPrivate::Tag::HasTargetMarker | !ElementPrivate::Tag::Target;
  d->setColor(mainColor());
}

QAction* Element::menuAction()
{
  Q_D(Element); Q_ASSERT(d);
  d->updatePersistentContextMenu();
  return d->persistentContextMenu.menuAction();
}



QGraphicsItem* Element::shapeItem()
{
  Q_D(Element); Q_ASSERT(d);
  return d->shapeItem;
}

bool Element::isShapeVisible() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->shapeItem->isVisible();
}

void Element::setShapeVisibility(const bool& visibility)
{
  Q_D(Element); Q_ASSERT(d);
  if( d->shapeItem->isVisible() != visibility )
  {
    d->shapeItem->setVisible(visibility);
    emit shapeVisibilityChanged(visibility);
  }
}

void Element::showShape()
{
  setShapeVisibility(true);
}

void Element::hideShape()
{
  setShapeVisibility(false);
}



QGraphicsItem* Element::convexHullItem()
{
  Q_D(Element); Q_ASSERT(d);
  return d->convexHullItem;
}

bool Element::isConvexHullVisible() const
{
  Q_D(const Element); Q_ASSERT(d);
  return d->convexHullItem->isVisible();
}

void Element::setConvexHullVisibility(const bool& visibility)
{
  Q_D(Element);Q_ASSERT(d);
  if( isConvexHullVisible() != visibility )
  { 
    d->convexHullItem->setVisible(visibility);
    emit convexHullVisibilityChanged(visibility);
  }
}

void Element::showConvexHull()
{
  setConvexHullVisibility(true);
}

void Element::hideConvexHull()
{
  setConvexHullVisibility(false);
}


void Element::setZValue(qreal z)
{
  Q_D(Element); Q_ASSERT(d);
  
  d->convexHullItem->setZValue(z);
  d->shapeItem->setZValue(z);
  
}


#endif // WITH_GRAPHICS

#include "element.moc"