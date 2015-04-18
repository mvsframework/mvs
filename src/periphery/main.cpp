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

#include "config.h"
#include "debugsupport.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QApplication>
  #include "GUI/mainnodewindow.h"
#else
  #include <QtCore/QCoreApplication>
#endif

#include <QtCore/QTimer>

#include "launcher.h"




int main(int argc, char **argv)
{
  
  setMessagePattern();
  
/** \internal
 * Creating the (Core)Application effectively instantiated the presence of 
 * (Q)Threads. Hence, creating it early ensures that QObjects have a defined
 * thread affinity (to the main GUI thread).
 */  
  
#ifdef WITH_GRAPHICS
  using Application = QApplication;
  QApplication app(argc, argv);
  qDebug() << "Application (Main) GUI Thread:" << QApplication::instance()->thread();
#else // WITH_GRAPHICS
  QCoreApplication  app(argc, argv);
  qDebug() << "Application (Main) Thread:" << QCoreApplication::instance()->thread();
#endif // WITH_GRAPHICS
  
  //NOTE: it is not necessary to register IDL messages. The Topic using them 
  // does this.
  qRegisterMetaType<NodeId>();
  qRegisterMetaType<ElementId>();
  qRegisterMetaTypeStreamOperators<NodeId>("NodeId");
  qRegisterMetaType<Node::Type>();
  qRegisterMetaType<DDS_InstanceHandle_t>();
  
  Launcher* launcher = nullptr;

//   try
//   {
    launcher = new Launcher(app.arguments());    
    QObject::connect(launcher, SIGNAL(deletedAll()),
                     &app, SLOT(quit()));
//   }
//   catch(int e)
//   {
//     std::cout << "The Launcher couldn't initialize. (Exception Nr. " << e << " )"<< std::endl;
//     return 0; 
//   }
  
#ifdef WITH_GRAPHICS
  // iterate through all the nodes the launcher knows of and create a 
  // window with the corresponding graphics for it.
  for( auto& node : launcher->nodes() )
  {
//     auto thread = new QThread;
    auto nodeWindow = new MainNodeWindow(node);
//     nodeWindow->moveToThread(thread);
//     thread->start();
    nodeWindow->show();
  }
#endif // WITH_GRAPHICS
  
  /** Start the main event loop (must be called from the same thread which 
   * invoked main() !) */
  int appReturnCode = app.exec();
  
  // some cleanup for valgrind.
  delete launcher;
  launcher = nullptr;
  
  return appReturnCode;
}