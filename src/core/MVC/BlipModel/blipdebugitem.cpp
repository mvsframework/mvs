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

#include "MVC/BlipModel/blipdebugitem.h"
#include "MVC/poditem.h"

#include "blip_p.h"

BlipDebugItem::BlipDebugItem(const BlipPrivate* blip, TreeItem* parent)
  : TreeItem(parent)
  ,d(blip)
{
  name = QVariant(QObject::tr("Debug"));
  comment = QVariant(QObject::tr("Additional information for debugging."));

  
//   fovPolyVisible = new PodItem<const bool>(pod_fovPolyVisible,QObject::tr("FOV Poly")
//     ,"boolean", QObject::tr("Visbility of the FOV polygon."));
//   appendChild(fovPolyVisible);

//   rfPolyVisible = new PodItem<const bool>(d->showRfPolygonItem,QObject::tr("R/F Poly")
//     ,"boolean", QObject::tr("Visbility of the R/F polygon."));
//   appendChild(rfPolyVisible);
  
  comConnectedToHost = new PodItem<const bool>(d->isConnectedToHost,QObject::tr("COM Connected")
    ,"boolean", QObject::tr("Has a working COM connection to the host."));
  appendChild(comConnectedToHost);
  
  rfRelayFlag = new PodItem<const bool>(d->isRfRelay,QObject::tr("R/F Relay")
    ,"boolean", QObject::tr("Provides a COM connection to another Node of interest."));
  appendChild(rfRelayFlag);
  
  closestObstacle = new PodItem<const ElementId>(d->closestObstacleId, QObject::tr("Closest Obstacle")
    ,"ElementId", QObject::tr("The ID of the Element which is the closest obstacle."));
  appendChild(closestObstacle);
  
  targetElement = new PodItem<const ElementId>(d->targetElementId, QObject::tr("Target")
    ,"ElementId", QObject::tr("The ID of the Element set as the target."));
  appendChild(targetElement);
  
}

BlipDebugItem::~BlipDebugItem()
{
  delete targetElement;
  targetElement = nullptr;
  
  delete closestObstacle;
  closestObstacle = nullptr;
  
  delete comConnectedToHost;
  comConnectedToHost = nullptr;

  delete rfRelayFlag;
  rfRelayFlag = nullptr;
  
//   delete rfPolyVisible;
//   rfPolyVisible = nullptr;
  
//   delete fovPolyVisible;
//   fovPolyVisible = nullptr;
}



QVariant BlipDebugItem::data(int column, int role) const
{
  if( role == Qt::DisplayRole )
  {
    switch( column )
    {
      case static_cast<int>(DataAspect::name) : return name;
      case static_cast<int>(DataAspect::value) : return QVariant();
      case static_cast<int>(DataAspect::comment) : return comment;
      case static_cast<int>(DataAspect::type) : return QVariant();
      default: return QVariant("(default)");
    }
  }
  //else
  return QVariant();
}
