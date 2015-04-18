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

#ifndef BLIPTREEVIEW_H
#define BLIPTREEVIEW_H

#include <QtWidgets/QTreeView>
#include <QtGui/QMouseEvent>
#include "nodeid.h"

class BlipTreeView : public QTreeView {
Q_OBJECT
public:
  BlipTreeView();
  virtual ~BlipTreeView();
  
Q_SIGNALS:
  /** \brief Indicates the request for a blip specific context menu.
   * \param[in] globalPoint The global point at which the context menu has been
   *  requested.
   * \param[in] id The NodeId of the blipd for which the context menu has been
   *  requested.
   */
  void blipContextMenuRequested(QPoint globalPoint, NodeId id);
  
protected:
  
  virtual void mouseMoveEvent(QMouseEvent* event);
  
private:
  /** \brief Get the id of the blip shown at point.
   * \param[in] point A local point within the widget
   * \param[out] id The NodeId of the blip associated with that point.
   * \return TRUE if the mapping was successfull and id is valid; FALSE 
   *  otherwise
   */
  bool mapToBlip(QPoint const & point, NodeId & id);

};

#endif // BLIPTREEVIEW_H
