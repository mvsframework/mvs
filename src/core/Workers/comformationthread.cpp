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

#include "comformationthread.h"

ComFormationThread::ComFormationThread(Node* parent)
  :QThread(parent)
{

}

ComFormationThread::~ComFormationThread()
{
  mutex.lock();
    abort=true;
    condition.wakeOne();
  mutex.unlock();

  wait();
}


void ComFormationThread::proposeComFormation(const ElementId& target, const NodeId& source)
{
  QMutexLocker locker(&mutex);
  
  this->target = target;
  this->source = source;
  
  if( !isRunning() )
  { start(); }
  else
  { 
    restart = true;
    condition.wakeOne();
  }
}

/** \internal Adopted from 
 * http://qt-project.org/doc/qt-5/qtcore-mandelbrot-example.html :
 * 
 * "run() is executed in its own thread (the 'worker' thread), whereas
 * the ComFormationThread constructor and destructor (as well as the 
 * porposeComFormationa() function) are called by the thread that created the 
 * worker thread. Therefore, we need a mutex to protect accesses to the abort 
 * and condition variables, which might be accessed at any time by run()."
 * 
 */
void ComFormationThread::run()
{
  forever{
    //
    // Get a fresh local copy of the input parameters inside the run() function
    // so that there will be only minimal code inside the mutex lock - just
    // the copy stuff...
    //
    
    mutex.lock();
      ElementId target = this->target;
      NodeId source = this->source;
    mutex.unlock();
  
    //
    // BEGIN of the actual work section of this thread
    // \/=============================================
    // 
    //
    
    //NOTE: While the actual work is done, two things should/need to be checked
    // more or less regularly in order to make the program responsive:
    //
    // * the 'restart' flag
    // * the 'abort' flag
    //
    // When the 'restart' flag is detected to be 'true', the remaining work 
    // should be skipped and the program execution should preferably be 
    // continuing _after_ the END of the actual work section (i.e. with the part
    // that locks 'restart', resets it, and goes back to the top of the forever
    // loop).
    // When the 'abort' flag is detected, the run() method should return so 
    // that the class can be deleted by the constructor.
    
    
    
    
    
    //
    // /\=============================================
    // END of the actual work section of this thread
    //
    
    mutex.lock();
    if (!restart)
    { // the thread is done with it's work and apparently not restarting, hence
      // put the thread to sleep
      condition.wait(&mutex);
    }
    else
    { 
      // apparently the worker needs to be restarted... set the restart flag to
      // false (so that we can capture the next restart again), and, well, 
      // restart the work... which happens automagically as the forever loop
      // will just continue on...
      restart = false;
    }
    mutex.unlock();
  } // end forever loop
}


// void ComFormationThread::test() 
// {
//   forever {
//         mutex.lock();
//         QSize resultSize = this->resultSize;
//         double scaleFactor = this->scaleFactor;
//         double centerX = this->centerX;
//         double centerY = this->centerY;
//         mutex.unlock();
//         int halfWidth = resultSize.width() / 2;
//         int halfHeight = resultSize.height() / 2;
//         QImage image(resultSize, QImage::Format_RGB32);
// 
//         const int NumPasses = 8;
//         int pass = 0;
//         while (pass < NumPasses) {
//             const int MaxIterations = (1 << (2 * pass + 6)) + 32;
//             const int Limit = 4;
//             bool allBlack = true;
// 
//             for (int y = -halfHeight; y < halfHeight; ++y) {
//                 if (restart)
//                     break;
//                 if (abort)
//                     return;
// 
//                 uint *scanLine =
//                         reinterpret_cast<uint *>(image.scanLine(y + halfHeight));
//                 double ay = centerY + (y * scaleFactor);
// 
//                 for (int x = -halfWidth; x < halfWidth; ++x) {
//                     double ax = centerX + (x * scaleFactor);
//                     double a1 = ax;
//                     double b1 = ay;
//                     int numIterations = 0;
// 
//                     do {
//                         ++numIterations;
//                         double a2 = (a1 * a1) - (b1 * b1) + ax;
//                         double b2 = (2 * a1 * b1) + ay;
//                         if ((a2 * a2) + (b2 * b2) > Limit)
//                             break;
// 
//                         ++numIterations;
//                         a1 = (a2 * a2) - (b2 * b2) + ax;
//                         b1 = (2 * a2 * b2) + ay;
//                         if ((a1 * a1) + (b1 * b1) > Limit)
//                             break;
//                     } while (numIterations < MaxIterations);
// 
//                     if (numIterations < MaxIterations) {
//                         *scanLine++ = colormap[numIterations % ColormapSize];
//                         allBlack = false;
//                     } else {
//                         *scanLine++ = qRgb(0, 0, 0);
//                     }
//                 }
//             }
// 
//             if (allBlack && pass == 0) {
//                 pass = 4;
//             } else {
//                 if (!restart)
//                     emit renderedImage(image, scaleFactor);
//                 ++pass;
//             }
//         }
//         
//         
//         
//         mutex.lock();
//         if (!restart)
//            condition.wait(&mutex);
//         restart = false;
//         mutex.unlock();
//     }
//   }
//   
// }
