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

#include "attitudeitem.h"

#include "MVC/poditem.h"
#include <QtCore/QObject> // for tr()

AttitudeItem::AttitudeItem(AMG::EulerAngleTupel const & eulerAngles
                          ,QString const & name, QString const & comment
                          ,TreeItem* parent)
  :TreeItem(parent)
  ,attitude(eulerAngles)
  ,name(name)
  ,comment(comment)
{
  phi = new  PodItem<const double>(attitude[0],"φ","°", QObject::tr("roll"));
  appendChild(phi);
  
  theta = new PodItem<const double>(attitude[1],"θ","°",  QObject::tr("pitch"));
  appendChild(theta);
  
  psi = new PodItem<const double>(attitude[2],"ψ","°", QObject::tr("yaw"));
  appendChild(psi);
}

AttitudeItem::~AttitudeItem()
{
  delete phi;
  phi = nullptr;
  
  delete theta;
  theta = nullptr;
  
  delete psi;
  psi = nullptr;
}

QVariant AttitudeItem::data ( int column, int role ) const
{
  if( role == Qt::DisplayRole )
  {
    switch( column )
    {
      case static_cast<int>(DataAspect::name)   : return name;
      case static_cast<int>(DataAspect::value)  : return QVariant(lookingAtString());
      case static_cast<int>(DataAspect::comment): return comment;
      case static_cast<int>(DataAspect::type)   : return QVariant();
      default: return QVariant("(default)");
    }
  }
  //else
  return QVariant();
}


QString AttitudeItem::lookingAtString() const
{
  QString compassPoint = AMG::GIS::convertToString(AMG::GIS::headingToCompassPoint(attitude[2])).c_str();
  return QObject::tr("Looking %1").arg(compassPoint);
}
