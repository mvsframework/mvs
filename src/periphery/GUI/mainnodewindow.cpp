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

#include "mainnodewindow.h"
#include "Graphics/palette.h"

#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QDockWidget>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtSvg/QSvgGenerator>

#include "Nodes/vehicle.h"
#include "Nodes/gustuav.h"
#include "Nodes/controlstation.h"

MainNodeWindow::MainNodeWindow( Node* const m_node, QWidget* parent )
  :QMainWindow(parent)
  ,m_node(m_node)
{
   setWindowTitle(tr("SAI").append(" - ").append(m_node->objectName())); //FIXME: Magic Numbers
   
   createScene();
   createCentralWidget();
   createMenus();
   createStatusBar();
   createDockWindows();
   
   qDebug() << objectName() << ": MainNodeWindow( Node* const m_node, QWidget* parent )"
    << "with parent =" << parent
    << "and thread affinity" << thread();
}

MainNodeWindow::~MainNodeWindow()
{

}


void MainNodeWindow::closeEvent ( QCloseEvent* event)
{
  m_node->halt();
  m_node->deleteLater();
  QMainWindow::closeEvent(event);
}


void MainNodeWindow::createMenus()
{
  QAction* action = nullptr;
  
  //
  // Interface Menu
  //
  {
  interfaceMenu = menuBar()->addMenu( tr("&Interface") );

  action = new QAction(this);
  action->setText( tr("&Close") );
  interfaceMenu->addAction(action);
  
  connect(action, SIGNAL(triggered()),
          this, SLOT(close()) );
  }
  
  //
  // Debug Menu
  //
  {
  debugMenu = menuBar()->addMenu( tr("&Debug") ); 
  
  auto simItemSlots = debugMenu->addMenu(tr("&SimItem"));
  
  action = new QAction(this);
  action->setText( tr("(Re-)&Initialize"));
  connect( action, SIGNAL(triggered()) ,
           m_node, SLOT(initialize())  );
  simItemSlots->addAction(action);
  
  action = new QAction(this);
  action->setText( tr("&Run"));
  connect( action, SIGNAL(triggered()) ,
           m_node, SLOT(run()) );
  simItemSlots->addAction(action);
  
  action = new QAction(this);
  action->setText( tr("&Halt"));
  connect( action, SIGNAL(triggered()) ,
           m_node, SLOT(halt()) );
  simItemSlots->addAction(action);
  
  
  auto nodeSlots = debugMenu->addMenu(tr("&Node"));
   
  action = new QAction(this);
  action->setText( tr("&Join"));
  connect( action, SIGNAL(triggered()) ,
           m_node, SLOT(joinSwarm()) );
  nodeSlots->addAction(action);
  
  action = new QAction(this);
  action->setText( tr("&Leave"));
  connect( action, SIGNAL(triggered()) ,
           m_node, SLOT(leaveSwarm()) );
  nodeSlots->addAction(action);
  
  
  
  auto vehicle = dynamic_cast<Vehicle*>(m_node);
  if( vehicle  != nullptr )
  {
    auto vehicleSlotsMenu = debugMenu->addMenu(tr("&Vehicle"));
    
    action = new QAction(this);
    action->setText( tr("Enable &Joystick Control"));
    connect( action, SIGNAL(triggered()) ,
            vehicle, SLOT(goJoystickControl()) );
    vehicleSlotsMenu->addAction(action);
    
    action = new QAction(this);
    action->setText( tr("&Stop Motion"));
    connect( action, SIGNAL(triggered()) ,
            vehicle, SLOT(stopMotion()) );
    vehicleSlotsMenu->addAction(action);

    action = new QAction(this);
    action->setText( tr("&Continue Motion"));
    connect( action, SIGNAL(triggered()) ,
            vehicle, SLOT(continueMotion()) );
    vehicleSlotsMenu->addAction(action);
    
        vehicleSlotsMenu->addSeparator();
    
    action = new QAction(this);
    action->setText( tr("goTo(&random)"));
    connect( action, SIGNAL(triggered()) ,
            vehicle, SLOT(goTo_randomPointInArena()) );
    vehicleSlotsMenu->addAction(action);
  }
  
  auto gustUav = dynamic_cast<GustUav*>(m_node);
  if( gustUav != nullptr )
  {
    auto gustUavSlotsMenu = debugMenu->addMenu(tr("&GUST UAV"));
    
    action = new QAction(this);
    action->setText( tr("traj&Go"));
    connect( action, SIGNAL(triggered()) ,
            gustUav, SLOT(gust_trajGo()) );
    gustUavSlotsMenu->addAction(action);
    
    action = new QAction(this);
    action->setText( tr("traj&Stop"));
    connect( action, SIGNAL(triggered())   ,
            gustUav, SLOT(gust_trajStop()) );
    gustUavSlotsMenu->addAction(action);
    
    action = new QAction(this);
    action->setText( tr("&Panic Stop"));
    connect( action, SIGNAL(triggered())   ,
            gustUav, SLOT(gust_trajPanicStop()) );
    gustUavSlotsMenu->addAction(action);
  }
  
  
  auto controlStation = dynamic_cast<ControlStation*>(m_node);
  if( controlStation != nullptr )
  {
    auto csSlotsMenu = debugMenu->addMenu(tr("&ControlStation"));
    
    action = new QAction(this);
    action->setText( tr("Realize &Formation"));
    connect( action, &QAction::triggered ,
            controlStation, &ControlStation::realizeFormation );
    csSlotsMenu->addAction(action);
      
  }
    
#if defined DEBUG_GRAPHICS  
{    
  /** \todo Some of these debug graphic items can have their visibility changed
   * in other spots and that is not back propagated to the related actions.
   * As such it can be that the actual visibility and the status of the action
   * checkbox do not match, requiring the user to use the action twice. */
  
  
  auto debugGraphicsMenu = debugMenu->addMenu(tr("Debug &Graphics"));
      
  action = new QAction(this);
  action->setText( tr("&VRONI Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isVroniGraphVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setVroniGraphVisibility(checked); } );
      
  action = new QAction(this);
  action->setText( tr("&Face Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isFaceGraphVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setFaceGraphVisibility(checked); } );
      
  action = new QAction(this);
  action->setText( tr("Face-to-Obstacle &Association"));
  action->setCheckable(true);
  action->setChecked(m_node->isFaceAssociationVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setFaceAssociationVisibility(checked); } );
  
  action = new QAction(this);
  action->setText( tr("&Bifurcation Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isBifurcationGraphVisible());
  debugGraphicsMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setBifurcationGraphVisibility(checked); } );
  
  action = new QAction(this);
  action->setText( tr("Bifurcation &Edge Association"));
  action->setCheckable(true);
  action->setChecked(m_node->isBifurcationAssociationVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setBifurcationAssociationVisibility(checked); } );
      
  action = new QAction(this);
  action->setText( tr("&Obstacle Bifurcation Edge Association"));
  action->setCheckable(true);
  action->setChecked(m_node->isObstacleEdgeAssociationVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setObstacleEdgeAssociationVisibility(checked); } );
      
  action = new QAction(this);
  action->setText( tr("Obstacle RouteGraph &Vertex Association"));
  action->setCheckable(true);
  action->setChecked(m_node->isObstacleVertexAssociationVisible());
  debugGraphicsMenu->addAction(action);
  
  connect( action, &QAction::triggered,
    [=](bool checked){ m_node->setObstacleVertexAssociationVisibility(checked); } );
  
    
  action = new QAction(this);
  action->setText( tr("&R/F Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isRfGraphVisible());
  debugGraphicsMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setRfGraphVisibility(checked); } );
    
  action = new QAction(this);
  action->setText( tr("&Directed R/F Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isDirectedRfGraphVisible());
  debugGraphicsMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setDirectedRfGraphVisibility(checked); } );
} 
#endif // DEBUG_GRAPHICS
  
    
  
//   
//   action = new QAction(this);
//   action->setText( tr("&Join Network"));
//   connect( action, SIGNAL(triggered()) ,
//            m_node, SLOT(joinSwarm()) );
//   commandMenu->addAction(action);
//   
//   action = new QAction(this);
//   action->setText( tr("&Leave Network"));
//   connect( action, SIGNAL(triggered()) ,
//            m_node, SLOT(leaveSwarm()) );
//   commandMenu->addAction(action);
//   
//   action = new QAction(this);
//   action->setText( tr("&Shut Down"));
//   connect( action, SIGNAL(triggered()) ,
//            m_node, SLOT(shutdown()) );
//   commandMenu->addAction(action);

  }
  
  //
  // View Menu
  //
  {
  viewMenu = menuBar()->addMenu( tr("&View") );
  
  action = new QAction(this);
  action->setText( tr("&Route Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isRouteGraphVisible());
  viewMenu->addAction(action);
  
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setRouteGraphVisibility(checked); } );

  
  action = new QAction(this);
  action->setText( tr("&COM Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isComGraphVisible());
  viewMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setComGraphVisibility(checked); } );


  action = new QAction(this);
  action->setText( tr("&Formation Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isFormationGraphVisible());
  viewMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setFormationGraphVisibility(checked); } );
  
  
  action = new QAction(this);
  action->setText( tr("&Obstacle Graph"));
  action->setCheckable(true);
  action->setChecked(m_node->isObstacleGraphVisible());
  viewMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setObstacleGraphVisibility(checked); } );

  
  action = new QAction(this);
  action->setText( tr("&Obstacle Cells"));
  action->setCheckable(true);
  action->setChecked(m_node->areObstacleCellsVisible());
  viewMenu->addAction(action);
    
  connect( action, &QAction::triggered,
           [=](bool checked){ m_node->setObstacleCellVisibility(checked); } );
    

//NOTE: This doesn't work as during initial construction there is no target 
// environment and hence the call to isTargetEnvironmentVisible causes a SIGABRT
//  
//   action = new QAction(this);
//   action->setText( tr("&Target Environment"));
//   action->setCheckable(true);
//   action->setChecked(m_node->isTargetEnvironmentVisible());
//   viewMenu->addAction(action);
//     
//   connect( action, &QAction::triggered,
//            [=](bool checked){ m_node->setTargetEnvironmentVisibility(checked); } );
    
  
  
  viewMenu->addSeparator();
  
  
  action = new QAction(this);
  action->setText( tr("&Export to SVG"));
  viewMenu->addAction(action);
  
  connect( action, &QAction::triggered,
           this, &MainNodeWindow::saveViewAsSvg );
  }
}

void MainNodeWindow::createScene()
{
  scene = new GraphicsScene(this);
  
  m_node->setScene(scene);
}

void MainNodeWindow::createCentralWidget()
{
  view = new GraphicsView(scene,this);
  setCentralWidget(view);
}

void MainNodeWindow::createDockWindows()
{

  QDockWidget* dock = nullptr;
 
  //
  // Node Tree View
  //
  
  blipTreeView = new BlipTreeView;
  blipTreeView->setModel(m_node->blipModel());
  
  dock = new QDockWidget(tr("Blips"),this);
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea );
  dock->setWidget(blipTreeView);
  
  connect(blipTreeView,SIGNAL(blipContextMenuRequested(QPoint,NodeId)),
          this, SLOT(showBlipContextMenu(QPoint,NodeId)));
    
  addDockWidget(Qt::LeftDockWidgetArea, dock);
  viewMenu->addSeparator();
  viewMenu->addAction(dock->toggleViewAction());
  
  
}


void MainNodeWindow::createStatusBar()
{
  statusBar()->showMessage(tr("Hello."));
  
//   connect(scene, &GraphicsScene::cursorPos,
//     [=](QPointF pos){
//       statusBar()->showMessage(QString("Scene Pos.: (x: %1,y: %2)").arg(pos.x()).arg(pos.y()));
//     }
//   );
}


void MainNodeWindow::showBlipContextMenu ( QPoint point, NodeId id )
{
  QMenu menu;
  m_node->buildBlipContextMenu(&menu,id);
  menu.exec(point);
}

void MainNodeWindow::showElementContextMenu(QPoint point, ElementId id)
{
  QMenu menu;
  m_node->buildElementContextMenu(&menu,id);
  menu.exec(point);
}

/** \todo This seems to always save the complete scene, albeit scaled depending 
 * on the zoom level of the view. */
void MainNodeWindow::saveViewAsSvg() const
{
  QString fileName;
  fileName += QDir::currentPath(); 
  fileName += "/SAI Map "; //FIXME: Magic Numbers: Preamble of file name for exported SVGs.
  fileName += QDateTime::currentDateTime().toString();
  fileName += ".svg";
  
  qDebug() << "\"MainNodeWindow\" : Exporting SVG as:" << fileName;
  
  QSvgGenerator svgGenerator;  
  svgGenerator.setFileName(fileName);
  svgGenerator.setSize(view->size());
  
  QPainter painter;
  painter.begin(&svgGenerator);
  view->render(&painter);
  painter.end();
}




#include "mainnodewindow.moc"