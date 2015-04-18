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


#ifndef DEBUGSUPPORT_H
#define DEBUGSUPPORT_H

#include <QtCore/qglobal.h>
#include <QtCore/QString>

// #define CONSOLE_COLOR 1 // use colors in console outputs
// #define CONSOLE_COLOR 0 // don't use color in console outputs
#include "config.h" // for DEBUG_CONSOLE_COLORS


/** \brief Set the message pattern used by Qt message handlers.
 * \details This sets the pattern used by qDebug(), qWarning(), qCritical(), and
 * qFatal().
 * See http://qt-project.org/doc/qt-5/qtglobal.html#qSetMessagePattern */
void setMessagePattern() {
  
//   const QString debug     = "[D]";
//   const QString warning   = "\033[1m[W]\033[0m";       /// [W] in bold
//   const QString critical  = "\033[31;1m[C]\033[0m";   /// [C] in bold red
//   const QString fatal     = "\033[31;47;1;7m[F]\033[0m"; /// [F] in bold white on red
//   const QString defaultPattern = "%{if-category}%{category}: %{endif}%{message}";
  
  
#ifdef DEBUG_CONSOLE_COLORS  
  const QString releasePattern = "%{if-debug}[D] %{endif}%{if-warning}\033[1m[W]\033[0m %{endif}%{if-critical}\033[31;1m[C]\033[0m %{endif}%{if-fatal}\033[31;47;1;7m[F]\033[0m %{endif}%{message}";
  const QString debugPattern = "%{if-debug}[D] %{endif}%{if-warning}\033[1m[W]\033[0m %{endif}%{if-critical}\033[31;1m[C]\033[0m %{endif}%{if-fatal}\033[31;47;1;7m[F]\033[0m %{endif}%{message}\t[%{function}@%{line}]";
#else  
  const QString releasePattern = "%{if-debug}[D] %{endif}%{if-warning}[W] %{endif}%{if-critical}[C] %{endif}%{if-fatal}[F] %{endif}%{message}";
  const QString debugPattern = "%{if-debug}[D] %{endif}%{if-warning}[W] %{endif}%{if-critical}[C] %{endif}%{if-fatal}[F] %{endif}%{message}\t[%{function}@%{line}]";
#endif   
  
#ifndef NDEBUG // NOTE: double negative... 8-|
  qSetMessagePattern(debugPattern);
#else // DEBUG
  qSetMessagePattern(releasePattern);
#endif // DEBUG
  
};



#endif // DEBUGSUPPORT_H
