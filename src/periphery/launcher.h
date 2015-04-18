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

#ifndef LAUNCHER_H
#define LAUNCHER_H

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtCore/QHash>

// musc headers  

#include "XSD/sai.hxx"          
#include "Geometry/geometry.h"
#include "Nodes/node.h"       


/** \brief A Launcher processes settings and initializes the sim.
 */
class Launcher : public QObject
{
  Q_OBJECT

Q_SIGNALS:
  /** \brief Indicating that all related Node entities have been deleted.
   * \sa deleteAll()
   */
  void deletedAll();
  
public Q_SLOTS:
  /** \brief Delete all created Node entities. 
   * \details Emits deletedAll when done.
   */
  void deleteAll();
  
public:
  /** \brief Constructor.
   * 
   * The list of arguments to the simulation executable:
   * \arg \c --settings= \< \a fileName \c \> The (absolute or relative) qualified 
   *  filename of the XML file holding the settings for this run. This defaults
   *  to \c ./< \a name_of_executable \c >.xml .
   * 
   * \param[in] args A \c --argname=argvalue string list
   */
  Launcher(QStringList const & args);
  
  /** \brief Destructor. */
  virtual ~Launcher();
  
  
  QList<Node*> nodes() const
  { return nodeList; };
  
private:
  Q_DISABLE_COPY(Launcher)
  
  
  /** \brief The values of the command line arguments. 
   * Some noteable parameters are
   * - settings = the path to the XML file containing the main settings.
   */
  QHash<QString, QString> arguments; 
  
  /** \brief A representation of the loaded XML settings.
   * 
   * \internal From http://www.cplusplus.com/reference/memory/auto_ptr/ :
   * 
   * [The auto_ptr] template provides a limited garbage collection facility for 
   * pointers, by allowing pointers to have the elements they point to 
   * automatically destroyed when the auto_ptr object is itself destroyed.
   * auto_ptr objects have the peculiarity of taking ownership of the pointers 
   * assigned to them: An auto_ptr object that has ownership over one element 
   * is in charge of destroying the element it points to and to deallocate the 
   * memory allocated to it when itself is destroyed. The destructor does this 
   * by calling operator delete automatically.
   * 
   * Therefore, no two auto_ptr objects should own the same element, since both 
   * would try to destruct them at some point. When an assignment operation 
   * takes place between two auto_ptr objects, ownership is transferred, which 
   * means that the object losing ownership is set to no longer point to the 
   * element (it is set to the null pointer).
   */
  std::auto_ptr<XML::SAI::Configuration> xmlSettings;
  
  /** \brief The (non-simple) polygon describing the operational arena
   * The outer ring of the polygon represents the boundary for all motion,
   * the inner rings eventually will represent obstacles.
   * 
   * \note The arena is a 2D polygon in the datum frame, i.e. the x-coordinates
   *   represent North, the y-coordinates East (assuming the datum is a NED
   *   frame).
   */
  Geometry::Polygon_2D arena;
  
  /** \brief A list of Obstacles extracted from the KML. */
  QList<Obstacle> obstacleList;
  
  
  /** \brief A list of all nodes LaunchControl has instanciated.
   * This is a combination of other (more specific) lists
   * \sa uavNodes 
   * \sa controlStationNodes
   */
  QList<Node*> nodeList;
   
  /** \brief Initialize the argument list for the LaunchControl
   * This functions processes the args QStringList of the Launcher
   * constructor and splits it into the QHash arguments  so that 
   * "--parameter = value" is stored as arguments["parameter"]=value;
   * 
   * \note This function runs QString::simplified() on parameter and value 
   *  before storing it into the hash.
   */
  void initializeArguments(QStringList args);

  /** \brief Initialize the Frames of Reference.
   * Creates an ECEF frame as a root frame and initilizes a datum frame as 
   * defined in the \<DatumNED\> XML element.
   * 
   * These frames are accessible through AMG::CoSy::getECEF() and 
   * AMG::CoSy::getDatumFrame(), resp.
   * 
   * \pre xmlSettings is initialized.
   */
  void initializeFrames();
  
  /** \brief Load the operational boundary.
   * Loads the outer operational boundary from the \c \<Arena\> XML element. This 
   * is used as the maximum visible area (i.e. without any obstacles, this area 
   * is visible to the nodes) as well as the maximum motion area (i.e. no node 
   * can leave this area).
   * 
   * \pre The AMG datum frame is initialized.
   */
  void loadBoundary();
  
  
  /** \brief Convert a (Lon,Lat,Alt) string into a Point with datum coordinates
   * \param[in] coordinateString A Lat,Lon,Alt string in degrees and meters
   * \return A point with the corresponding coordinates in the Datum frame.
   */
  Geometry::Point_2D datumPointFromKmlString(const QString& pointCoordinateString,
                                             const QString& coordinateSeperator = ",");
  
  
  /** \brief Import polygonal obstacles from a KML-like document.
   * 
   * Loads a KML formatted file. All polygons in that have a \c obstacle:category
   * attribute are individually imported into the obstacleList.
   * 
   * The expected KML format is this:
   * \verbatim
     <Placemark obstacles:category="building">
       <Polygon>
         <outerBoundaryIs>
           <LinearRing>
             <coordinates>
               <!-- longitude,latitude,altitude longitude,latitude,altitude ... -->
             </coordinates>
           </LinearRing>
         </outerBoundaryIs>
       </Polygon>
     </Placemark>
     \endverbatim
   *
   * \note This function does not check for duplicates when importing!  
   * 
   * \param[in] kmlFileName The (relative or qualified) file name of the KML 
   *  file to parse.
   * \return The collection of created polygons
   */
  QList<Obstacle> loadKmlPolygons(std::string const & kmlFileName,
                                                    QString const & pointSeperator=" ",
                                                    QString const & coordinateSeperator = ",");
  
  
  /** \brief Load obstacles.
   * Loads obstacles from the the \c \<Obstacles\> XML element and combines 
   * them into the arena polygon.
   * 
   * \pre Assumes that all obstacles are non-intersecting, simple and inside 
   *   the arena (i.e., inside the \c \<Arena\> XML element) 
   */
  void loadObstacles();
  
  /** \brief Construct and initialize the individual nodes of the simulation.
   * Initilizes nodes based on the data in the \c \<Nodes\> XML element, which 
   * could look like this:
   * \code
<UAV>
  <name>UAV 0</name>
  <coordinates>-84.38456511651836,33.75917337933702,40.0</coordinates>
  <startupDelay>200</startupDelay>
  <Gust>
    <gustIp>192.168.54.150</gustIp>
    <gustUdpPort>3010</gustUdpPort>
    <localIp>192.168.54.150</localIp>
    <localUdpPort>2010</localUdpPort>
    <inpFilePath>~/svn/work/rmax</inpFilePath>
    <manAugment>
      <anom>30</anom>
      <jerk>20</jerk>
      <vcHzGo>10</vcHzGo>
      <vcHzStop>20</vcHzStop>
      <vcHzSide>5</vcHzSide>        
    </manAugment>
  </Gust>
</UAV>
  \endcode
   */
  void initializeNodes();

  
};

#endif // LAUNCHER_H
