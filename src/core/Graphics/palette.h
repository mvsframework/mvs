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

#ifndef PALETTE_H
#define PALETTE_H

#include <QtCore/QHash>

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QColor>

#include "IDL/data.h"

/** \addtogroup Graphics
 *  @{
 */ 

namespace Graphics_2D { 



// namespace Palettes {
//   
// } // end namespace Graphics_2D::Palettes


namespace Brushes {
  
  class FloodFill : public QBrush {
  public:
    /** \brief Constructor.
      * \param[in] color The color of the fill.
      * \param[in] opacity (optional) The opacity in percent, i.e. [0,100]
      */
    FloodFill(QColor const& color, int const& opacity = 0)
    :QBrush(color,Qt::SolidPattern)
    { 
      if( opacity != 0)
      {
        QColor c = color;
        c.setAlphaF(qBound(0,opacity,100)/100.0);
        setColor(c);
      }
    };
    ~FloodFill() = default;
  };
   
  extern QBrush const rfPolygon;
 
} //end namespace Graphics_2D::Brushes

namespace Pens {
  
  class Outline : public QPen {
  public:
    Outline(QColor const color=Qt::black)
    :QPen(color,0,Qt::SolidLine){};
    ~Outline() = default;
  };

  class DashedOutline : public QPen {
  public:
    DashedOutline(QColor const color=Qt::black)
    :QPen(color,0,Qt::DashLine){};
    ~DashedOutline() = default;
  };
  
  extern QPen const rfPolygon;
  
}  //end namespace Graphics_2D::Pens

/** \internal Qt::GlobalColor enum:
  * 
  * Qt::color0        0  0 pixel value (for bitmaps)
  * Qt::color1        1  1 pixel value (for bitmaps)
  * Qt::black         2  Black         (#000000)
  * Qt::white         3  White         (#ffffff)
  * Qt::darkGray      4  Dark gray     (#808080)
  * Qt::gray          5  Gray          (#a0a0a4)
  * Qt::lightGray     6  Light gray    (#c0c0c0)
  * Qt::red           7  Red           (#ff0000)
  * Qt::green         8  Green         (#00ff00)
  * Qt::blue          9  Blue          (#0000ff)
  * Qt::cyan         10  Cyan          (#00ffff)
  * Qt::magenta      11  Magenta       (#ff00ff)
  * Qt::yellow       12  Yellow        (#ffff00)
  * Qt::darkRed      13  Dark red      (#800000)
  * Qt::darkGreen    14  Dark green    (#008000)
  * Qt::darkBlue     15  Dark blue     (#000080)
  * Qt::darkCyan     16  Dark cyan     (#008080)
  * Qt::darkMagenta  17  Dark magenta  (#800080)
  * Qt::darkYellow   18  Dark yellow   (#808000)
  * Qt::transparent  19  a transparent black value (i.e., QColor(0, 0, 0, 0))
  */
namespace Colors {
  
  extern QColor const comGraph;
  extern QColor const routeGraph;
  extern QColor const obstacleGraph;
  extern QColor const activeFlightplan;  
  extern QColor const inactiveFlightplan;  
  extern QColor const targetElement;
  extern QColor const nonTargetElement;
  
  /** \brief Get a (pseudo) random color. 
   * \todo Randomize the seed. */
  inline QColor randomColor()
  {
    auto r = qrand() % 255;
    auto g = qrand() % 255;
    auto b = qrand() % 255;
    
    return QColor::fromRgb(r,g,b);
  };
  
  /** \brief Get a color from a list of 12 distinguishable colors.
   * 
   * The set was obtainted from Color Brewer 2.0 (http://colorbrewer2.org/)
   * 
   * \param[in] index A 0-based index for the colors; wraps around through the
   * use of a modulo operator.
   * \return The respective QColor.
   */
  static QColor colorBrewerSet(uint const index)
  {  
    switch( index % 12){
      case 0 : return QColor::fromRgb( 0xa6,0xce,0xe3 );
      case 1 : return QColor::fromRgb( 0x1f,0x78,0xb4 );
      case 2 : return QColor::fromRgb( 0xb2,0xdf,0x8a );
      case 3 : return QColor::fromRgb( 0x33,0xa0,0x2c );
      case 4 : return QColor::fromRgb( 0xfb,0x9a,0x99 );
      case 5 : return QColor::fromRgb( 0xe3,0x1a,0x1c );
      case 6 : return QColor::fromRgb( 0xfd,0xbf,0x6f );
      case 7 : return QColor::fromRgb( 0xff,0x7f,0x00 );
      case 8 : return QColor::fromRgb( 0xca,0xb2,0xd6 );
      case 9 : return QColor::fromRgb( 0x6a,0x3d,0x9a );
      case 10: return QColor::fromRgb( 0xff,0xff,0x99 );
      case 11: return QColor::fromRgb( 0xb1,0x59,0x28 );
      default: return QColor::fromRgb( 0x0, 0x0, 0x0  );
    }
  };
  
  
}


struct Utensils {
  QBrush brush;
  QPen pen;
};    

/** \brief A Palette is a combination of a brush and a pen for different states. */
using Palette = QHash<IDL::Data::StateDescriptorId,Utensils>;


} // end namespace Graphics_2D

/** @}*/

#endif // PALETTE_H