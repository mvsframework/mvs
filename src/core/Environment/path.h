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

#ifndef PATH_H
#define PATH_H

#include <QtCore/QObject>
#include <QtCore/QScopedPointer>
#include <QtCore/QMetaProperty>

#include "config.h" // provides WITH_GRAPHICS

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsItem>
#endif // WITH_GRAPHICS

class PathPrivate; // Forward declaration.

/** \brief A generic class describing paths through the environment. 
 * \todo Well, technically this should be a graph at its core. Practically all 
 * I need right  * now is "a context menu" for a graph graphic...
 */
class Path : public QObject
{
Q_OBJECT
  
public:
  /** \brief Default constructor. */
  Path(QObject* parent = nullptr);
  
  /** \brief Destructor. */
  virtual ~Path();
   
#ifdef WITH_GRAPHICS

  QGraphicsItem* graphicsItem() const;
  void setGraphicsItem(QGraphicsItem* item);
#endif // WITH_GRAPHICS
  
protected:
  QScopedPointer<PathPrivate> const d_ptr;
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Path( PathPrivate& dd, QObject* parent = 0 );
  
  
#ifdef WITH_GRAPHICS
  virtual void  contextMenuEvent(QGraphicsSceneContextMenuEvent * event);
#endif // WITH_GRAPHICS  

  
private:
  Q_DISABLE_COPY(Path)
  Q_DECLARE_PRIVATE(Path)
  
};

#endif // PATH_H
