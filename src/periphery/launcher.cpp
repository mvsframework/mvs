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

#include "launcher.h"
#include "config.h"

#include <QtCore/QFile>
#include <QtCore/QDebug>
#include <QtCore/QTimer>
#include <QtCore/QThread>

#include <iostream>
#include <amg.hpp>


// musc headers 

#include <XSD/xmltools.h>
#include <Nodes/node.h>
#include <Nodes/controlstation.h>

#include "uav.h"

Launcher::Launcher(QStringList const & args)
{
  setObjectName("Launcher");
  
  initializeArguments(args);
  
  if( QFile::exists(arguments["settings"]) )
  { // try loading the settings XML file into a C++ representation
//     try
//     {
      xmlSettings = XML::SAI::configuration(arguments["settings"].toStdString());
//     }
//     catch(const xml_schema::Exception& e)
//     {
//       Q_CHECK_PTR( e.what() );
//       std::cerr << "error in Launcher(QStringList const &)"<< std::endl;
//       std::cerr << e.what() << std::endl;
//     }
    
    initializeFrames();
    loadBoundary();
    loadObstacles();
    initializeNodes();
    
    qDebug() << objectName() << ": Enjoy the show.\n\n\n";
    
  }
  else
  { // error handling
    //NOTE: qFatal() will _abort_ on a UNIX system!
    qFatal("%s: Cannot load settings from %s. (File does not seem to exist.)"
      ,objectName().toStdString().c_str()
      ,arguments["settings"].toStdString().c_str());
  }
}

Launcher::~Launcher()
{
#ifndef WITH_GRAPHICS
  deleteAll(); // When there are graphics, thw MainWindow deletes the node.
#endif // WITH_GRAPHICS
}

void Launcher::initializeArguments(QStringList args)
{
  qDebug() << objectName() << ": Processing argument list:" << args;
    
  if( args.length() <= 1 ) //NOTE: the name of the executable is always the first "argument"
  {
    qDebug() << objectName() << ": No arguments detected. Using defaults:";
    
    // Get the name of the executable in order to generate a default settings 
    // file name
    QString defaultSettingsFileName = args.takeFirst().split("/").takeLast().append(".xml");
    arguments["settings"]= QString("XML/").append(defaultSettingsFileName);
    qDebug() << objectName() << ": --settings =" << arguments["settings"];
   
    return;
  }
  
  
  /** \todo Make this argument parser more in line with other linux parameters,
   * i.e. change "--parammeter=value" to "--parameter value". The later would 
   * allow for tab completion on input files, etc., but the arguemtn count would
   * double in the below foreach loop. */
  
  QString argument;
  foreach( argument, args) 
  {
    QStringList arg=argument.split("=");
    
    if (    arg.length() == 2 
        and arg[0].length() > 2
        and arg[0].startsWith("--")  )
    {
      QString parameter = arg[0].remove(0,2).simplified();
      QString value = arg[1].simplified();
      qDebug() << objectName() << ": Parameter" << parameter << "=" << value;
      arguments[parameter] = value; 
    }
   
  }
}

void Launcher::initializeFrames()
{
  Q_ASSERT( xmlSettings.get() != nullptr );
  
  qDebug() << objectName() << ": Initializing Frames...";

  // generate the root frame of reference as the ECEF

  auto ecefFrame = new AMG::FrameOfReference; // Let there be an Origin...

  // store an accessor to the ECEF frame, accessable with AMG::CoSy::getECEF()
  AMG::CoSy::setECEF(ecefFrame);
  
  
  /** \todo Numerically it might make more sense to auto determine the datum
   * based upon the (later to be loaded) boundary information.
   */
  

  
  XML::SAI::Configuration::DatumNEDType& datum = xmlSettings->datumNED();
  if( datum.coordinates().present() )
  {
  
    // The Coordinates_t type is a <list itemType="xs:string" />, hence the 
    // trouble with the iterators even though the datum _should_ only have one entry!
    // Compare:
    // http://www.codesynthesis.com/pipermail/xsd-users/2007-February/000816.html
    // http://www.codesynthesis.com/pipermail/xsd-users/2007-October/001295.html
    XML::SAI::Coordinates_t& datumCoords = datum.coordinates().get();
    XML::SAI::Coordinates_t::iterator itCoords = datumCoords.begin(); 
      
    // KML specifies its coordinates as a string list:
    // <coordinates>lon,lat,alt lon,lat,alt ...</coordinates>
    // The XML list type is delimited by the "space" in between the 3-tupels of 
    // individual coordinates for a 3D location...
    QStringList coordinates(QString(std::string(*itCoords).c_str()).split(",")); //FIXME: magic numbers (coordinates seperator in XML)

    //... so after we have gotten the first list entry, furhter splitting is needed:
    double lonGeod_deg = coordinates[0].toDouble();
    double latGeod_deg = coordinates[1].toDouble();
    double elevGeod_m  = coordinates[2].toDouble();
    
    if( elevGeod_m != 0.0 )
    {
      qDebug() << objectName() << ": The non-zero altitude (" << elevGeod_m << "m ) of the NED Datum frame is ignored."; 
    }
          
    auto datumFrame = AMG::CoSy::newNorthEastDown<AMG::Units::degree>(lonGeod_deg,latGeod_deg);

    // store an accessor to the datum frame, access with AMG::CoSy::getDatumFrame()
    // All AMG::RigidBodies will default to be defined in this frame
    AMG::CoSy::setDatumFrame(datumFrame);  
  } 
}

void Launcher::loadBoundary()
{
  using namespace Geometry;
  
  qDebug() << objectName() << ": Loading the operational boundary...";
  Q_ASSERT(AMG::CoSy::getDatumFrame());
  
  XML::SAI::Configuration::ArenaType& arenaXml = xmlSettings->arena();
  if( arenaXml.coordinates().present() )
  {
    Polygon_2D boundaryPolygon;

    // The Coordinates_t type is a <list itemType="xs:string" />, hence the 
    // trouble with the iterators.
    // Compare:
    // http://www.codesynthesis.com/pipermail/xsd-users/2007-February/000816.html
    // http://www.codesynthesis.com/pipermail/xsd-users/2007-October/001295.html
    XML::SAI::Coordinates_t& arenaCoords = arenaXml.coordinates().get();   
    XML::SAI::Coordinates_t::iterator itCoords =arenaCoords.begin(); 
    
    while( itCoords != arenaCoords.end() ) 
    {
      
      Coordinates_Geodetic lonLatAlt_deg( kmlStringToPointCoordinate(*itCoords) );
      Coordinates_ECEF ecefCoordiantes( geodetic2ecef(lonLatAlt_deg) );
      Coordinates_Datum datumCoordinates( ecef2datum(ecefCoordiantes) );
      
      Point_2D point(get<0>(datumCoordinates),
                     get<1>(datumCoordinates));
      
      boost::geometry::append(boundaryPolygon, point);
      
      // proceed to the next point
      ++itCoords;
    }
    
    boost::geometry::correct(boundaryPolygon);  
    
    arena = boundaryPolygon;
    
    std::cout << "Arena : " << boost::geometry::dsv(arena) << std::endl;
    std::cout << "Arena area: " << boost::geometry::area(arena) << " m²" << std::endl;
    
  } // end if( arena.coordinates().present() )
}

Geometry::Point_2D Launcher::datumPointFromKmlString(const QString& pointCoordinateString,
                                                     const QString& coordinateSeperator)
{
  using namespace Geometry;
  
  Coordinates_Geodetic lonLatAlt_deg( kmlStringToPointCoordinate(pointCoordinateString,coordinateSeperator) );
  Coordinates_ECEF ecefCoordiantes( geodetic2ecef(lonLatAlt_deg) );
  Coordinates_Datum datumCoordinates( ecef2datum(ecefCoordiantes) );
  
  return Point_2D(get<0>(datumCoordinates),
                  get<1>(datumCoordinates));
}

QList<Obstacle> 
Launcher::loadKmlPolygons(const std::string& kmlFileName, const QString& pointSeperator, const QString& coordinateSeperator)
{
  using namespace Geometry;
  QDomDocument kmlDocument;
  
  QList<Obstacle> obstacleList;

  if( XML::loadXmlFile(kmlFileName, kmlDocument))
  {
    // get all placemarks from the document
    QDomNodeList placemarkList = kmlDocument.elementsByTagName("Placemark"); //FIXME: magic text
 
    // iterate through the found placemarks
    int numberOfFoundKMLPlacemarks = placemarkList.size();
    for(int i=0; i<numberOfFoundKMLPlacemarks; ++i)
    { 
      QDomElement placemark =placemarkList.item(i).toElement();
      if( "building"==placemark.attribute("obstacles:category") ) //FIXME: magic text
      {
        // get the " " separated list of geodetic coordinates of the corners
        auto kmlCoordsString = placemark
                      .firstChildElement("Polygon")
                      .firstChildElement("outerBoundaryIs")
                      .firstChildElement("LinearRing")
                      .firstChildElement("coordinates")
                        .text().simplified();
       
        // create the corresponding polygon
        Polygon_2D polygon;
  
        auto pointStringList = QString(kmlCoordsString).split(pointSeperator);
        QString pointCoordinateString;
        foreach( pointCoordinateString, pointStringList)
        {
          Point_2D point(datumPointFromKmlString(pointCoordinateString, coordinateSeperator));
          boost::geometry::append(polygon,point);                        
        }

        boost::geometry::correct(polygon); 
        
        Obstacle obstacle;
        obstacle.setShape(polygon);
        
        auto nameElement = placemark.firstChildElement("name");
        if( !nameElement.isNull() )
        { obstacle.setObjectName( nameElement.text().simplified() ); }
        else
        { obstacle.setObjectName( tr("Obstacle")); }
        
//         qDebug() << objectName() << ":" << obstacle.objectName();
        
        obstacleList.push_back(obstacle);
      }
    }
  }
  
  qDebug() << objectName() << ": Done.";
  
  return obstacleList;
}

void Launcher::loadObstacles()
{
  qDebug() << objectName() << ": Importing obstacles...";
  
  using namespace Geometry;
  
  obstacleList = loadKmlPolygons(xmlSettings->obstacles().href());
  
  int obstacleCount = 0;
  for( Element & element : obstacleList )
  {
      ElementId id = obstacleCount++; 
      Q_ASSERT(id != NULL_ELEMENT_ID);
      element.setId(id); 
  }
  Q_ASSERT(obstacleCount == obstacleList.count());
  
     
  qDebug() << objectName() << ": Loaded" << obstacleCount << "obstacles from"
    << XML::qString(xmlSettings->obstacles().href()) ;
}

void Launcher::initializeNodes()
{
  qDebug() << objectName() << ": Constructing nodes...";
  
  XML::SAI::NodeGroup_t&  xmlNodes = xmlSettings->nodes();
  
  using StartupPair = QPair<Node*, XML::SAI::GenericNode_t::StartupDelayType>;
  QList<StartupPair> startup;
  
  // iterate over all the sub-elements in <Nodes> ... </Nodes>
  XML::SAI::NodeGroup_t::AbstractNodeIterator it_node = xmlNodes.abstractNode().begin();
  while( it_node != xmlNodes.abstractNode().end() )
  {
    Node* node = nullptr;
    
    
    // check for the type of the abstract node iterator and work through the
    // instantiation
    if( Uav::Settings* uavXmlSettings = dynamic_cast< Uav::Settings*> (&(*it_node)))
    { // Uav Node
      if( !node )
      { node = new Uav( uavXmlSettings ); }
    }
    
    if( ControlStation::Settings* csXmlSettings = dynamic_cast<ControlStation::Settings*>(&(*it_node)))
    { // ControlStation Node
      if( !node )
      { node = new ControlStation( csXmlSettings ); }
    }
    
    if( Node::Settings* nodeXmlSettings = dynamic_cast< Node::Settings*> (&(*it_node)))
    { // Generic Node
      if( !node )
      { node = new Node( nodeXmlSettings ); }
      
//       QThread* thread = new QThread(this);
//       { // try to move each node into its own thread...
//         qDebug() << "Attempting to move" << static_cast<QObject*>(node) 
//           << "to thread" << thread << "...";
//          node->moveToThread(thread);
//          thread->start();
//          qDebug() << "...done? (" << static_cast<QObject*>(node)  <<"is in thread" << node->thread() <<")";
//       }
      
            
      nodeList.append(node);
      node->setOperationalBoundary(arena.outer());
      
      for( auto& obstacle : obstacleList )
      { 
//         qDebug() << objectName() << ": added obstacle" << obstacle.objectName();
//         obstacle.moveToThread(thread);
        node->addObstacle(obstacle);
      }
        
      node->updateArena();  
      
      if( nodeXmlSettings->startupDelay().present() )
      { startup.append( StartupPair(node,nodeXmlSettings->startupDelay().get()) ); } 
    } 
    else 
    {
      qCritical() << objectName() << ": Node type detecetion of XML instance failed.";
    }
    
    
    
    ++it_node; // advance to the next abstract node.
    
  }
  qDebug() << objectName() << ": Done constructing.";
   
  // initialize the individual nodes
  foreach( Node* node, nodeList )
  {
    
    qDebug() << objectName() << ": Initializing" << node->objectName();
    node->initialize();
   
  }
  qDebug() << objectName() << ": Done initializing.";
 
  if( startup.size() > 0 )
  {
    Node* node = nullptr;
    StartupPair pair;
    foreach( pair, startup)
    {
      node = pair.first; 
      auto delay = pair.second;
     
      qDebug() << objectName() 
        << ": Scheduling" << node->objectName() << "to start in" << delay << "ms.";
      QTimer::singleShot(delay, node, SLOT(run()));
    }
    node = nullptr;
  }
  
}

void Launcher::deleteAll()
{
  qDebug() << objectName() << ": Deleting all Nodes.";
  for( QList<Node*>::iterator it = nodeList.begin();
               it != nodeList.end(); it++ )
  {
    Node*& node = *it;
    // This check is necessary as deleteAll() could have been called previously
    // (and now it could be called from ~Launcher())
    if(node != nullptr)
    {
      node->halt();
      node->deleteLater();
      node = nullptr;
    }
  }
    
//   delete AMG::CoSy::getECEF(); //TODO: do i need to set something to NULL?
  
  emit deletedAll();
}

#include "launcher.moc"