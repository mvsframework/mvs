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

#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H

#include <QtWidgets/QGraphicsScene>

#include "nodeid.h"
#include "elementid.h"

class GraphicsScene : public QGraphicsScene {
Q_OBJECT  
Q_SIGNALS:
  void cursorPos(QPointF /*mousePosition*/);
  
public:
  /** \brief Delegating Constructor. */
  GraphicsScene ( QObject* parent = 0 )
    :GraphicsScene(QRectF(),parent){};
    
  /** \brief Delegating Constructor. */
  GraphicsScene ( qreal x, qreal y, qreal width, qreal height, QObject* parent = 0 )
    :GraphicsScene(QRectF(x,y,width,height),parent){};
    
  /** \brief Constructor. */
  GraphicsScene ( const QRectF& sceneRect, QObject* parent = 0 );
    
  virtual ~GraphicsScene()=default;
  
//   void requestBlipContextMenu(QPoint point, NodeId id);
//   void requestElementContextMenu(QPoint point, ElementId id);

protected:
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
  
private:
  Q_DISABLE_COPY(GraphicsScene)

};

#endif // GRAPHICSSCENE_H
