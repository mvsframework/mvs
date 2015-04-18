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

#include "MVC/BlipModel/positionitem.h"
#include "MVC/poditem.h"
#include <boost/graph/graph_concepts.hpp>
#include <QtCore/QObject> // for tr()
#include <amg.hpp>

PositionItem::PositionItem( const AMG::Vector& vector, const QString & name
  , const QString & comment, TreeItem* parent )
  :TreeItem(parent)
  ,position(vector)
  ,name(name)
  ,comment(comment)
  ,type()
{
  x = new PodItem<const double>(position.coords(0),"x","m", QObject::tr("North (wrt. Datum)"));
  appendChild(x);

  y = new PodItem<const double>(position.coords(1),"y","m", QObject::tr("East (wrt. Datum)"));
  appendChild(y);
  
  z = new PodItem<const double>(position.coords(2),"z","m", QObject::tr("Down (wrt. Datum)"));
  appendChild(z);
}

PositionItem::~PositionItem()
{
  delete x;
  x = nullptr;

  delete y;
  y = nullptr;
  
  delete z;
  z = nullptr;
}

QVariant PositionItem::data ( int column, int role ) const
{
  if( role == Qt::DisplayRole )
  {
    switch( column )
    {
      case static_cast<int>(DataAspect::name) : return name;
      case static_cast<int>(DataAspect::value) : return QVariant(relativePositionString());
      case static_cast<int>(DataAspect::comment) : return comment;
      case static_cast<int>(DataAspect::type) : return QVariant();
      default: return QVariant("(default)");
    }
  }
  //else
  return QVariant();
}


QString PositionItem::geodeticPositionString() const
{
  return QString(AMG::GIS::geodeticPositionString(position).c_str());
}

QString PositionItem::relativePositionString() const
{
  AMG::Vector posVector = position;
  const double & x = posVector.coords(0);
  const double & y = posVector.coords(1);
  
  double distance2d_xy = std::sqrt(x*x+y*y);
  
  double heading_rad = std::atan2(y,x); // heading_rad is in (-pi,pi]
  double heading_deg = AMG::Units::radian2degree(heading_rad); // heading_deg is in (-180,180]

  QString compassPoint = AMG::GIS::convertToString(AMG::GIS::headingToCompassPoint(heading_deg)).c_str();
  
  return QString("%1 m %2 (%3°)")
          .arg(distance2d_xy,0,'f',0)
          .arg(compassPoint)
          .arg(heading_deg,0,'f',0);

}

