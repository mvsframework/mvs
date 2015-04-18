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

#include "velocityitem.h"
#include "MVC/poditem.h"
#include <QtCore/QObject> // for tr()
#include <cmath>
#include <amg.hpp>

VelocityItem::VelocityItem( const AMG::Vector& velocity, const QString & name
                          , const QString & comment, TreeItem* parent )
  :TreeItem(parent)
  ,velocity(velocity)
  ,name(name)
  ,comment(comment)
{
  xDot = new PodItem<const double>(velocity.coords(0),"ẋ","m/s");
  appendChild(xDot);
  
  yDot = new PodItem<const double>(velocity.coords(1), "ẏ","m/s");
  appendChild(yDot);
  
  zDot = new PodItem<const double>(velocity.coords(2), "ż","m/s");
  appendChild(zDot);
}

VelocityItem::~VelocityItem()
{
  delete xDot;
  xDot = nullptr;
  
  delete yDot;
  yDot = nullptr;
  
  delete zDot;
  zDot = nullptr;
}

QVariant VelocityItem::data ( int column, int role ) const
{
  if( role == Qt::DisplayRole )
  {
    switch( column )
    {
      case static_cast<int>(DataAspect::name)   : return name;
      case static_cast<int>(DataAspect::value)  : return QVariant(speedString());
      case static_cast<int>(DataAspect::comment): return comment;
      case static_cast<int>(DataAspect::type)   : return QVariant();
      default: return QVariant();
    }
  }
  //else
  return QVariant();
}

/** \todo Make use of the already computed heading and speed in the blip
 *  secondary data.
 */
QString VelocityItem::speedString() const
{
  double speed = velocity.norm();
  
  if( speed > 0.1 ) //FIXME: Magic Numbers
  {
    double heading_deg = AMG::Units::radian2degree(std::atan2(velocity.coords(1),velocity.coords(0)));
    QString compassPoint = AMG::GIS::convertToString(AMG::GIS::headingToCompassPoint(heading_deg)).c_str();
    
    return QObject::tr("%1 m/s %2 (%3°)")
          .arg(speed,0,'f',1) 
          .arg(compassPoint)
          .arg(heading_deg, 0,'f', 0);
  }
  // else
  return QObject::tr("---");
}

