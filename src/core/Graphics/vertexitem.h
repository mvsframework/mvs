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

#ifndef VERTEXITEM_H
#define VERTEXITEM_H


#include <QtWidgets/QGraphicsEllipseItem>

namespace Graphics_2D {

// template<typename Vertex>  
class VertexItem : public QGraphicsEllipseItem {
public:  
  VertexItem( qreal x, qreal y, qreal dia = 1, QGraphicsItem* parent = nullptr)
  :QGraphicsEllipseItem(x-dia/2,y-dia/2,dia,dia,parent)
  {};
    
  void setDiameter(qreal dia)
  { setRect(pos().x()-dia/2,pos().y()-dia/2,dia,dia); };
  
  qreal diameter() const 
  { return rect().width(); }
};  

}

#endif // VERTEXITEM_H