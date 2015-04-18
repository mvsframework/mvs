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

#ifndef MAINNODEWINDOW_H
#define MAINNODEWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>

#include "Nodes/node.h"
#include "graphicsscene.h"
#include "graphicsview.h"
#include "GUI/bliptreeview.h"

class MainNodeWindow : public QMainWindow {
Q_OBJECT
public:
  /** \brief Constructor.
   * \param[in] node The node this window will be representing.
   * \param[in] parent (optional) The parent of this window.
   */
  MainNodeWindow(Node* const node,QWidget* parent=nullptr);
  virtual ~MainNodeWindow();
  
public Q_SLOTS:
  /** \brief Show a Blip-specific context menu.
   * \param[in] point The global point where to show the menu.
   * \param[in] id The ID of the Blip for which the menu should be shown.
   */
  void showBlipContextMenu(QPoint point, NodeId id);
  
  /** \brief Show an Element-specific context menu.
   * \param[in] point The global point where to show the menu.
   * \param[in] id The ID of the Element for which the menu should be shown.
   */
  void showElementContextMenu(QPoint point, ElementId id);
  
protected:
  virtual void closeEvent ( QCloseEvent* event);

private:
  Node* const m_node; ///< \brief The node this window is representing
  
  QMenu* interfaceMenu  = nullptr;
  QMenu* debugMenu      = nullptr;
  QMenu* viewMenu     = nullptr;
  void createMenus(); ///< \brief Create the menu bar for the window.
  
  
  GraphicsScene* scene = nullptr;
  void createScene();
  
  GraphicsView* view = nullptr;
  void createCentralWidget();
  
  BlipTreeView* blipTreeView = nullptr;
  void createDockWindows(); ///< \brief Create the side panel docks.
  
  void createStatusBar();
  
  void saveViewAsSvg() const;
};


#endif // MAINNODEWINDOW_H
