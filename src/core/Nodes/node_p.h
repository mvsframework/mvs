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

#ifndef NODEPRIVATE_H
#define NODEPRIVATE_H

#include <unordered_map>
#include <limits>
#include <climits>
#include <assert.h>

#include <QtCore/QMutex>
#include <QtCore/QFutureWatcher>

#include <boost/graph/planar_face_traversal.hpp> // compare http://www.boost.org/doc/libs/1_52_0/libs/graph/doc/planar_face_traversal.html
#include <QtCore/qshareddata.h>

#include "simitem_p.h"
#include "nodeid.h"
#include "location.h"
#include "StateMachine/activity.h"
#include "IDL/idlsupport.h"
#include "DDS/broadcasttopic.h"
#include "DDS/ddssupport.h"
#include "Database/table.h"
#include "DDS/datalink.h"
#include "vroniinterface.h" // also provides RouteGraph
#include "Graph/formationgraph.h"
#include "Graph/comgraph.h"
#include "Graph/elementgraph.h"
#include "Graph/visibilitygraph.h"
#include "Graph/routegraph.h"
#include "Graph/bifurcationgraph.h"
#include "blip.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsScene>
  #include <QtWidgets/QGraphicsItem>
  
  #include "Graphics/coordinatesystemitem.h"
  #include "Graphics/geometryitem.h"
  #include "Graphics/palette.h"
  #include "Graphics/arena.h"
  #include "Graphics/graphitem.h"
  #include "Graphics/palette.h"

  #include "MVC/BlipModel/blipmodel.h"
  #include "blip_p.h"
  
#ifdef DEBUG_GRAPHICS
  #include "Graph/directedrfgraph.h"
#endif // DEBUG_GRAPHICS

#endif // WITH_GRAPHICS


class Node;

class NodePrivate : /*public QObject, */public SimItemPrivate
{
  Q_ENUMS(RfZone)
  
private:
  Q_DECLARE_PUBLIC(Node);
  Node* const q_ptr;  
  
public:
  NodePrivate( Node* q
              ,Node::Settings* const settings
              /*,SimItem* parent*/ );
  virtual ~NodePrivate();
  
  Node::Settings* const xmlSettings; // initialized in construction list

  QMutex mutex;
  QMutex mutex_graphics_comGraph;
  
  QHash<DDS_InstanceHandle_t, NodeId> detectedParticipants;

  
  /** \name Node-specific states for the SimItem's state machine. */ //@{
 
  State*    standby           = nullptr;
  State*    active            = nullptr; ///< \brief (ParallelStates)
  State*    active_nodeRegion = nullptr; ///< \brief A Qt necessity to reflect the item's orthogonal region.
  Activity* joining           = nullptr;
  State*    operational       = nullptr;
  Activity* leaving           = nullptr;

  //@}
  
  NodeId ownshipId;
  Datalink* dds = nullptr; 
  Datalink::Settings* ddsSettings = nullptr;
  
  /** \brief Initialize property map interfaces to externally stored data. */
  void initializePropertyMapInterfaces();
  
  
  /** \name Location Database */ //@{
  
  /** \brief The "database" holding all the Location related data. */  
  std::map<LocationId, Location> locationData;
  
  /** \brief A boost::property_map \e interface to Location data.
   * \sa locationData */
  boost::associative_property_map<
      std::map<LocationId, Location>
    > location_pMap;  
   
  /** \brief Create and register a new Location.
   * \details makeLocation creates a new Location, registers it in 
   * locationData and returns a reference to the new Location. */
  inline Location& makeLocation()
  { 
    Location location;
    locationData[location.id()] = location;
    return locationData[location.id()];
  };
  /** \brief Create and register a new Location.
   * \overload
   * \param[in] position The position of the new location. */
  Location& makeLocation(Geometry::Point_2D const& position)
  { 
    Location location(position);
    if( locationData.count(location.id()) == 0 )
    { locationData[location.id()] = location; }
    else
    { qCritical() << "Location already exists, returning referecen to existing Location."; }
    return locationData[location.id()];
  };;  
    
  //@}  
    
    
    
  /** \brief The "database" holding all the Blip related data. */
  std::unordered_map<NodeId,Blip*> blipData;
  
  /** \brief A boost::property_map \e interface to Blip data.
   * \sa blipData */
  boost::associative_property_map<
      std::unordered_map<NodeId,Blip*>
    > blip_pMap;

  /** \brief The "database" holding all the Obstacle related data. */
  std::unordered_map<ElementId,Obstacle> obstacleData;
 
  /** \brief A boost::property_map \e interface to Obstacle data.
   * \sa obstacleData */
  boost::associative_property_map<
      std::unordered_map<ElementId,Obstacle> 
    > obstacle_pMap;
  
  /** \brief The "database" holding the Obstacle related graph face data.
   * \note The face associated with the motion boundary is stored under the key
   * \c motionBoundaryElementId .
   */
  std::unordered_map<ElementId,Geometry::Polygon_2D> obstacleDelaunayFaceData;
  
  
#ifdef WITH_GRAPHICS
  /** \brief The parent item for all obstacle cell graphics.
   * \details This is the parent item for all obstacleDelaunayFaceGraphics items
   * which allows to manipulate them as a group.
   */
  QGraphicsItemGroup* obstacleCellGraphics = nullptr;
  
  
  /** \brief The graphics representing the obstacle cell faces.
   * \details The underlying geometries are stored in obstacleDelaunayFaceData */
  std::unordered_map<ElementId, QAbstractGraphicsShapeItem*> obstacleDelaunayFaceGraphics;
#endif //WITH_GRAPHICS  
  
  /** \brief The "database" holding the route graph vertices surrounding the 
   * obstacle.
   * \internal A std::set was chosen so that no particular care has to be taken
   * when inserting elements as a set doesn't allow duplicate entries.
   */
  std::unordered_map<ElementId,std::set<RouteGraph::Vertex_Descriptor>> obstacleRouteVertexData;
  
  /** \brief The "database" holding the bifurcation graph edges surrounding the obstacle.
   * \details This data is updated in NodePrivate::updateDerivedGraphs. */
  std::unordered_map<ElementId,std::set<BifurcationGraph::Edge_Descriptor>> obstacleBifurcationEdgeData;
  
  /** \brief The "database" holding the bifurcation graph vertices surrounding the obstacle.
   * \details This data is updatde in NodePrivate::updateDerivedGraphs. */
  std::unordered_map<ElementId,std::set<BifurcationGraph::Vertex_Descriptor>> obstacleBifurcationVertexData;
  
  /** \brief The "database" holding the route graph edges surrounding the obstacle.
   * \internal Isn't this a DEBUG cludge? The bifurcation edge data and the 
   * associated bifurcation location data \e should hold the same data...
   */
  std::unordered_map<ElementId,std::set<RouteGraph::Edge_Descriptor>> obstacleRouteEdgeData; 
  
  
  /** \brief The geometry of bounding the motion of all Nodes. It forms the 
   * outer ring of the environment. */
  Geometry::Polygon_2D motionBoundary;
  
  /** \brief The ElementId used to replicate the motion boundary in an Obstacle 
   * context.
   * \details The motion boundary isn't really an obstacle, hence it is not 
   * represented in the obstacle database -- which in turn means that it doesn't
   * have an Obstacle::Id associated with it. This explicit variable holds an
   * ElementId that can be used to identify the motion boundary. */
  ElementId motionBoundaryElementId = NULL_ELEMENT_ID;

  /** \brief The LocationId associated with the motion boundary, located at its 
   * centroid. */
  LocationId motionBoundaryLocationId = NULL_LOCATION_ID;
  
  const unsigned int motionBoundaryObstacleVertexIndex = std::numeric_limits<unsigned int>::max();
  
  /** \name Environment */ //@{
  
  /** \brief The 2D environment for the Node to operate in.
   * \sa environmentItem  */
  Geometry::Polygon_2D environment2D;
  
  
  /** \brief (Re-)set the environment for this Node.
   * \details This sets the environemnt polygon and triggers a recomputation of
   * the route and derived graphs (if the Node is already initilized).  */
  void setEnvironment(Geometry::Polygon_2D const & newEnvironment);
  
#ifdef WITH_GRAPHICS
  /** \brief The polygon representing the 2D environment.
   * \details The Node has ownership of this memory and needs to delete it.
   * \sa initializeGraphics()
   * \sa environment2D
   */
  Graphics_2D::Arena* environmentItem = nullptr; 
  
  Graphics_2D::Polygon* motionBoundaryItem = nullptr; 
#endif // WITH_GRAPHICS  
  
  
  
  bool environmentBoundaryIsCurrent = false;  
  bool environmentObstaclesAreCurrent = false;
  
  //@}
  
  /** \name Target Environment */ //@{
  
  /** \brief The combination of the target obstacle cell and its neighbors.
   * \sa targetEnvironmentGraphics
   * \sa comCorridor
   */
  Geometry::Polygon_2D targetEnvironment; 
 
#ifdef WITH_GRAPHICS   
  /** \brief The graphics representing the local target environment. */
  Graphics_2D::Polygon* targetEnvironmentGraphics = nullptr;
#endif // WITH_GRAPHICS
  
  bool isTargetEnvironmentValid = false;
  
  /** \brief Create the target environment around the indicated Element.
   * \details
   * - Get the target obstacle and locate it in the obstacle graph.
   * - Get the one-hop neighbor obstacles
   * - Join all obstacle cells to form the outer target environment polygon
   * - Do some checking to remove numerical artefacts
   * - Add the obstacles as holes
   * - store the resulting polygon in targetEnvironment
   * \sa unsetTargetEvronment()
   */
  void createTargetEnvironment(ElementId const& targetId);
  
  void unsetTargetEnvironment();
  
  //@}
  
  
  /** \name COM Corridor */ //@{
  
  /** \brief The environment to used for visibility computations supporting the 
   * COM connection to the target.
   * \details This is currently from the ownship to the target. 
   * \sa comCorridorGraphics
   * \sa targetEnvironment */
  Geometry::Polygon_2D comCorridor;
  
#ifdef WITH_GRAPHICS    
  /** \brief The graphics representing the local COM corridor. */
  Graphics_2D::Polygon* comCorridorGraphics = nullptr;
#endif // WITH_GRAPHICS
  
  bool isComCorridorValid = false;
  
  /** \brief Create/update the COM corridor to the indicated Element.
   * 
   * \todo The target is clear, that is given as a parameter. The source, 
   * however, is implicitely assumed to be the ownship - but it should be
   * the commanding control station, shouldn't it?
   * 
   * \sa unsetComCorridor()
   * \sa comCorridor */
  void createComCorridor(ElementId const& targetId, NodeId const& source);

  void computeCellBasedComCorridor(ElementId const& targetId, NodeId const& source);
  void computeSlotBasedComCorridor(ElementId const& targetId, NodeId const& source);
  void unsetComCorridor();
   
  //@}
  
  
  /** \name Path Computation */ //@{
   
  /** A structure to collect the data related to COM formation computations.
   * \sa proposeComFormation
   * \sa processComFormationProposal
   */
  struct PathComputationData
  { 
    using IngressMap = std::unordered_map< LocationId, std::list<LocationId>>;
    using LoopMap = std::unordered_map< LocationId, std::list< std::list< LocationId >>>;
    
    /** A mutex to protect access to the members of this struct. 
     * Lock it while interacting with this struct's members. */
    QMutex mutex; 
    
    /** The ID of the obstacle that is selected as the target. */
    ElementId target; 
    
    /** The ID of the Node that seeks to be connected to the target. */
    NodeId source; 
    
    /** The ingress locations to (and including) a bifurcation point
     * of a target cell, keyed by the bifurcation's LocationId. */
    IngressMap ingress; 
    
    /** The loops arount the target cell, keyed by the LocationId of the 
     * bifurcation point they originate from */
    LoopMap loops; 

    /** The watcher item used to trigger the processing of the COM formation 
     * proposal once all the loops are computed. */
    QFutureWatcher<void> watcher; 
    
    
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
     DirectedRfGraph::Graph directedRfGraph;
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
    
  };
  
  PathComputationData* comFormationData = nullptr;
   
  /** \brief A flag to indicate whether currently a computation of a COM 
   * proposal is running. */
  bool computingComFormationProposal = false; 
  
  /** \brief Compute the ingress portion of the COM formation to the target. 
   * 
   * \sa computeComLoopsAroundTargetObstacle
   * \sa combineIngressAndLoops
   * 
   * \param[in] data The data set to work with.
   */
  void computeComIngressToTargetCell(PathComputationData* data) const;
   
  /** \brief Compute minimum hop routes from the source Node to the target
   * cell intersection points.
   * 
   * \details The method computes an undirected visibility graph on all route
   * graph vertices (and an introduced extra vertex at the source Node's 
   * location) within the COM corridor.
   * This is then followed by a breadth-first search starting at the source 
   * location, which in turn is used to compute a set of minimum hop paths
   * to all the route graph interesection vertices around the target (if they
   * are within the COM corridor!), effectively generating the minimup-hop paths
   * between the source Node's location and the intersection vertices of the 
   * target cell.
   * 
   * \note There is no vetting of paths other than hop count, i.e. no visibility 
   * robustness, no link-goodness, nothing but hop count.
   * \note The minimum hop paths are not unique and not necessarily identical
   * among calls to this method!
   * \pre isRfGraphValid == true
   * 
   * \param[in] source The NodeId of the Node to be used as a source for the 
   * search.
   * 
   * \return The minimum hop Location sequence to reach the route graph 
   * intersection vertices identified by the LocationId used as key.
   */
  std::unordered_map< LocationId, std::list<LocationId>>
    minimumHopCountComRouteToTargetCell(NodeId const& source) const;

  
    
  /** \brief Compute the loop portion of the COM formation around the target. 
   * \sa computeComIngressToTargetCell
   * \sa combineIngressAndLoops
   * \return 
   */
  void computeComLoopsAroundTargetObstacle(PathComputationData* data);

  /** \brief Combine the ingress and loops sequences and prepare them for 
   * scorging.
   * \details Using the bifurcation points around the target as keys, the 
   * ingress section and the loops are combined (at that particular bifurcation
   * vertex). The individual combinations can then be scored and a "best" one 
   * picked.
   * 
   * \note The intrinsic sorting of a multimap should ensure that the/a 
   * combination with the lowest score should be available as the first entry.
   * 
   * \sa computeComIngressToTargetCell
   * \sa computeComLoopsAroundTargetObstacle
   * \sa scoreFormation
   * 
   * \param[in] ingress Paths to the various bifurcation points.
   * \param[in] loops Closed circular loops orginating at the various 
   * bifurcation points. 
   * \return A multimap containing the scored formations. 
   */
  std::multimap<double, RfGraph::Graph> combineIngressAndLoops(
        std::unordered_map< LocationId, std::list< LocationId > > ingressData, std::unordered_map< LocationId, std::list< std::list< LocationId > > > loopData
  );          
      
  /** \brief The lower the score, the better. 
   * \details Currently this simply uses the number of vertices in the formation 
   * as the score.  */
  double computeFormationScore(RfGraph::Graph formation);
  
  /** \brief Compute a proposed COM Formation around a given target. 
   * \sa proposedFormation
   * \pre comCorridor is correct
   * \param[in] target The Id of the target obstacle
   * \param[in] source The Node that should be the source for the proposed 
   * formation.
   * \return TRUE if the formation proposal process could be started; FALSE 
   * otherwise.
   */
  bool proposeComFormation(ElementId const& target, NodeId const& source);
  
  bool processComFormationProposal(PathComputationData*& data);
  
//   /** \brief The proposed COM formation around the target obstacle. */
//   std::pair<double, RfGraph::Graph> proposedComFormation;

  void unsetComFormation();
  
  
  /** \brief Get all Locations inside a specified region.
   * \details Currently a brute-force search that checks all known Locations for
   * inclusion insude the region.
   * \param[in] region A polygonal region covering the requested locations.
   * \return The Id's of the Locations inside the region.  */
  QList<LocationId> getLocations(Geometry::Polygon_2D const& region);
  
  
  QList<LocationId> getRouteLocations(Geometry::Polygon_2D const& region);
  QList<LocationId> getObstacleLocations(Geometry::Polygon_2D const& region);
  QList<LocationId> getBifurcationLocations(Geometry::Polygon_2D const& region);
  
   /** \brief A functor class providing a recursive loop detection algorithm. */
  template< typename Graph
          , typename DistanceMap

  //         , typename VertexIndexMap = typename boost::property_map<Graph,vertex_index_t>::type
  //         , typename EdgeIndexMap = typename boost::property_map<Graph,edge_index_t>::type
          >
  class LoopDetector {
  public:
  
    using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
    using Edge   = typename boost::graph_traits<Graph>::edge_descriptor;
    using Distance = typename DistanceMap::value_type;
    
    using VertexIndexMap = typename boost::property_map<Graph,boost::vertex_index_t>::type;
    using EdgeIndexMap = typename boost::property_map<Graph,boost::edge_index_t>::type;
  
  
    LoopDetector( Graph const& graph, DistanceMap const& distanceMap
                , NodePrivate* const parent) 
      :d(parent)
      ,graph(graph)
      ,dMap(distanceMap)
    {
      BOOST_CONCEPT_ASSERT(( boost::GraphConcept< Graph > ));
      BOOST_CONCEPT_ASSERT(( boost::ReadablePropertyMapConcept< DistanceMap, Edge > ));
    };
    
    virtual ~LoopDetector()
    {};
 
    /** \brief Detect all loops going through the indicated start vertex. 
     * \details Start a depth-first searach for loops in the graph, starting at
     * startVertex. The process terminates after an exhaustive search through
     * all branches through startVertex. Individual branch exploration is stoped
     * only if a loop is detected or the distance threshold or maximum hop count
     * is reached, whatever comes first.
     * \param[in] startVertex The starting point for the loop detection (all loops 
     * will have to pass through here)
     * \param[in] threshold When the accumulated distance surpasses this 
     * threshold the loop detection for that branch is stopped.
     * \param[in] d (optional) The initial distance to start with (this counts
     * towards the accumulated distance)
     * \param[in] maximumHopCount (optional) When a branch surpasses this hop 
     * count the loop detection for that branch is stopped.
     * 
     */
    void start(Vertex startVertex, Distance const& threshold, 
               Distance d = 0, unsigned int maximumHopCount = 5 )
    {
      dThreshold = threshold;
      
      detectedLoops.clear();
      maxHopCount = maximumHopCount;
      
      typename std::list<Vertex> list;
      recurse(startVertex,list,d);

//       qDebug() << "LoopDetector" 
//         << ": num_vertices =" << boost::num_vertices(graph)
//         <<", num_edges =" << boost::num_edges(graph);
//       qDebug() << "LoopDetector" << ": Detected loops:";
//       for( auto pair: detectedLoops )
//       { qDebug() << pair.first << "hops -"<< pair.second.size() << "loops"; }
    };
  
    /** \brief Detect all loops in the graph. 
     * \deprecated
     * \param[in] d (optional) The initial distance to start with (this counts
     * towards the accumulated distance)
     * \param[in] maximumHopCount (optional) When a branch surpasses this hop 
     * count the loop detection for that branch is stopped.
     * \fixme This detects the same loop several times: once for each involved 
     * node!
     */
    void start(Distance d = 0,  unsigned int maximumHopCount = 5)
    {
      detectedLoops.clear();
      maxHopCount = maximumHopCount;
      
      
      typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
      for( boost::tie(vi,vi_end)=boost::vertices(graph);vi!=vi_end;++vi )
      { 
        typename std::list<Vertex> list;
        recurse(*vi,list,d); 
      }
      qDebug() << "Detected loops:";
      for( auto pair: detectedLoops )
      { qDebug() << pair.first << "hops -"<< pair.second.size() << "loops"; }
    }
    
    /** \brief All detected loops.
     * \details The map is ordered by hop count as the key for each loop, the 
     * actual loop list(s) do not repeat the closing vertex. I.e. if the loop is 
     * {a b c d a}, the corresponding loop is filed in the 4-hop catergory and
     * the list contains {a,b,c,d}. */
    std::map<unsigned int, std::list<std::list<Vertex>>> detectedLoops;

  private:
    NodePrivate* const d;
    Graph const& graph;
    DistanceMap const& dMap;
    Distance dThreshold;
    
    unsigned int maxHopCount;
    
    
    void recurse(Vertex v, std::list<Vertex> path, Distance accumulatedDistance)
    {    
      path.push_back(v);
      
      if( path.size() <= maxHopCount )
      {
    //     qDebug() << "At Vertex" << boost::get(boost::vertex_index, graph, v) << ". Accumulated distance =" << accumulatedDistance;
      
      // iterate over all out edges
        typename Graph::out_edge_iterator ei, ei_end;
        for( boost::tie(ei, ei_end) = boost::out_edges(path.back(), graph); ei!=ei_end; ++ei )
        {
          Distance d = get(dMap,*ei);
          Distance newAccumulatedDistance = accumulatedDistance + d;
          
          Q_ASSERT( boost::source(*ei,graph)==v );
          
          Vertex nextVertex = boost::target(*ei,graph);
          
      //       qDebug() << "Taking Edge" << boost::get(boost::edge_index, graph, *ei) 
      //         << "to Vertex" << boost::get(boost::vertex_index, graph, nextVertex);
          
          if( newAccumulatedDistance <= dThreshold )
          {
            if( nextVertex == path.front() )
            { /* found a valid loop [path, nextVertex] */
      //           std::cout << "Found a loop:";
      //           for( auto vertex : path )
      //           { std::cout << " " <<boost::get(boost::vertex_index, graph, vertex); }
      //           std:: cout << " " << boost::get(boost::vertex_index, graph, nextVertex) << std::endl;

              detectedLoops[path.size()].push_back(path);
              
              continue; // there is no need to check any remaining edges leaving from path.back()
            }
            else
            { recurse(nextVertex,path,newAccumulatedDistance); }
          }
          else
          { /* no loop yet, but breached the threshold along [path, nextVertex]*/
      //         std::cout << "Reached the threshold. Stopping search." << std::endl;
      //           for( auto vertex : path )
      //           { std::cout << " " <<boost::get(boost::vertex_index, graph, vertex); }
      //           std:: cout << " " << boost::get(boost::vertex_index, graph, nextVertex) << std::endl;
          }
        }
      }
      else
      {
    //       std::cout << "Reached maximum hop count. Stopping search."<<std::endl;
      }
    };

  };
  

  //@}
  
  
  
  
#ifdef WITH_GRAPHICS
  
  /** \brief The scene this node is using to show its graphics.
   * \details This is set to an external scene, so this node does not have 
   * ownership of this memory.
   * \sa Node::setScene(QGraphicsScene*)
   */
  QGraphicsScene* scene = nullptr;
  
  /** \brief The root item for this Node's graphics "scene graph". 
   * \details The Node has ownership of this memory and needs to delete it.
   * \sa initializeGraphics()
   */
  Graphics_2D::CoordinateSystem* datumItem = nullptr;
  
  enum GraphicItemZLevels : int {
     DatumItemZLevel    = 0         ///< datumItem
    ,EnvironmentGraphicZLevel       ///< environmentItem
    ,ObstacleCellGraphicZLevel      ///< obstacleCellGraphics
    ,ObstacleGraphicZLevel
    
#ifdef DEBUG_GRAPHICS
    ,FaceToObstacleCorrespondenceGraphicZLevel ///< faceToObstacleCorrespondence
#endif // DEBUG_GRAPHICS    
    
    ,BlipRfPolygonZLevel
    ,OwnshipBlipRfPolygonZLevel
    ,BlipFovPolygonZLevel
    
    ,ComCorridorGraphicZLevel       ///< comCorridorGraphics
    
    
#ifdef DEBUG_GRAPHICS
    ,BifurcationGraphGraphicZLevel  ///< bifurcationGraphGraphics
    ,BifurcationEdgeAssociationGraphicZLevel
    ,VroniGraphGraphicZLevel        ///< vroniGraphGraphics //DEBUG Graphics
    ,VroniSiteShapeGraphicsZLevel
    ,VroniEdgeSiteGraphicsZLevel
    ,VroniEdgeShapeGraphicsZLevel
    ,FaceGraphGraphicZLevel         ///< faceGraphGraphics  //DEBUG Graphics
    ,ObstacleEdgeAssociationGraphicZLevel
#endif // DEBUG_GRAPHICS

    ,TargetEnvironmentGraphicZLevel ///< targetEnvironmentGraphics 
#ifdef DEBUG_GRAPHICS      
    ,RfGraphGraphicZLevel           ///< rfGraphGraphics 
    ,IngressGraphicZLevel           ///< ingressGraphics 
    ,ComLoopGraphGraphicZLevel      ///< comLoopGraphGraphics 
#endif // DEBUG_GRAPHICS
    ,FormationGraphGraphicZLevel    ///< formationGraphGraphics 
    ,ObstacleGraphGraphicZLevel     ///< obstacleGraphGraphics
    ,ComGraphGraphicZLevel          ///< comGraphGraphics
    ,RouteGraphZLevel               ///< routeGraphGraphics
    
    ,BlipCurrentFlightplanZLevel
    ,BlipCurrentWaypointZLevel
    ,BlipRfPolygonCenterZLevel
    ,OwnshipBlipRfPolygonCenterZLevel
    ,BlipBodyCarriedFrameZLevel
  };
  

  struct Palette {
    /** \brief Default (empty) constructor.
     * \details This constructor sets the Node-level defaults color schemes for 
     * all the used Pallets. */
    Palette();
    
    ::Blip::Palette blip;
    
    
//     /** \brief The pallet used for painting Blips.
//      * \details Depending on the type/state of the Blip, different pallets are 
//      * used to encode related information.
//      * (See Blip_PaintSchemes.pdf in the source tree.)
//      */
//     struct Blip {
//       Graphics_2D::Palette rfPolygon;
//     } relayBlip, nonRelayBlip, ownshipBlip;
    
    Graphics_2D::Utensils fovPoly_dark;
    Graphics_2D::Utensils fovPoly_light;
    
  } palette;
  
  
  BlipModel* blipModel = nullptr;
  
#endif // WITH_GRAPHICS
  
  /** \brief A Blip-wrapper that keeps primary data and DDS messages in sync.
   * 
   * As Blips are the main interface to handle data stored for Nodes, the 
   * OwnshipBlip provides two main benefits over the conventional Blip:
   * 
   * - Additional storage for data not kept for other nodes
   * - Automated syncinc of Blip-data and the related DDS messages.
   * 
   * \todo Implement PIMPL.
   */
  class OwnshipBlip : public Blip {
  public:
    OwnshipBlip(NodePrivate* const parent)
      :Blip(parent->q_func())
      ,parentNodePrivate(parent)
    {
#ifdef WITH_GRAPHICS
      {
        GraphicsParameter p;
        p.pastWpCount = -1; // show all past waypoints for the ownship
        p.futureWpCount = -1; // show all future waypoints for the ownship
        
        setParameter(p);        
      }
#endif // WITH_GRAPHICS
    };
    virtual ~OwnshipBlip() = default;
    
    virtual void setPosition3D ( const AMG::Vector& position );
    virtual void setAttitude ( const AMG::EulerAngleTupel& eulerAngles );
    virtual void setVelocity ( const AMG::Vector& velocity );
    
    /** \brief Set (and broadcast) the ownship's flight plan.
     * 
     */
    virtual bool setFlightplan ( const IDL::Data::Flightplan& idlPlan );
    
    /** \brief Set the ownship's target.
     * \details Set the target and update the status message; emitts the 
     * respective Node signals. */
    virtual void setTargetId( ElementId const & id );

#ifdef WITH_GRAPHICS    
    /** \brief Set the graphical item representing the Datum of the hosting Node. */
    virtual void setDatumItem(QGraphicsItem* const datumItem);
#endif // WITH_GRAPHCIS
    
    /** \brief Set (and broadcast) the ownship's flight plan to an empty one.
     * 
     * A convenience function for setFlightplan(IDL::Data::Flightplan());
     */
    virtual void clearFlightplan();
    
    /** \brief Set (and broadcast) the current waypoint index. */
    virtual bool setCurrentWaypointIndex(unsigned int const & index);
    
    
//     RfGraph::Vertex_Descriptor rfGraphVertex() const;
//     virtual void               setRfGraphVertex(RfGraph::Vertex_Descriptor const & descriptor);

  private:
    NodePrivate* const parentNodePrivate;
//     RfGraph::Vertex_Descriptor rfVertex; ///<\brief The R/F graph vertex representing the ownship.
  };
  OwnshipBlip* ownshipBlip = nullptr;
  
  /** \brief Hash to keep track of continuosly firing timer IDs . 
   * \details The key is the period of the beat in [ms].
   */
  QHash<unsigned int,int> metronome;
  
  /** \brief An interface to the VRONI library for Voronoi computations. */
  VroniInterface vroni;
 
  /** \name R/F Zones */ //@{
  
  /** \brief The range limits demarking different R/F distance zones.
   * \details Depending on the distance between to nodes, sevaral different 
   * zones can be defined. This struct holds the distance values that demark the
   * boundaries between those zones.
   * \sa RfZone
   * \sa correspondingRfZone
   *
   */
  struct RfZoneLimits {
    /** \brief The maximum R/F range of the Node in [m].*/
    static constexpr double MaxRfRange = 200;             //FIXME: Magic Numbers
    /** \brief The border between RfZone::Red and RfZone::Orange in [m].*/
    static constexpr double Alarm      = MaxRfRange-1.0;  //FIXME: Magic Numbers
    /** \brief The border between RfZone::Orange and RfZone::Yellow in [m].*/
    static constexpr double Warning    = MaxRfRange-5.0;  //FIXME: Magic Numbers
    /** \brief The border between RfZone::Yellow and RfZone::Green in [m]. */
    static constexpr double Caution    = Warning-15.0 ;   //FIXME: Magic Numbers
  };
  
  /** \brief An Enum indicating the R/F zones with decreasing "quality".
   * \sa correspondingRfZone
   * \sa RfZoneLimits
   */
  enum class RfZone : unsigned int {
//     Blue,
    /** \brief The R/F zone from \c 0 to RfZoneLimits::Caution.*/
    Green   = 4, 
    /** \brief The R/F zone from RfZoneLimits::Caution to RfZoneLimits::Warning.*/
    Yellow  = 3,
    /** \brief The R/F zone from RfZoneLimits::Warning to RfZoneLimits::Alarm */
    Orange  = 2,
    /** \brief The R/F zone from RfZoneLimits::Alarm to RfZoneLimits::MaxRfRange */
    Red     = 1,
    /** \brief The R/F zone outside RfZoneLimits::MaxRfRange. */
    White   = 0
  };

  /** \brief Get the R/F zone corresponding to a inter-Node distance.
   * \pre distance is greater or equal to zero.
   * \sa RfZone
   * \sa RfZoneLimits
   */
  static RfZone correspondingRfZone(double const & distance)
  {
    Q_ASSERT( distance >= 0.0 );
    if(                                         distance >= RfZoneLimits::MaxRfRange )
    { return RfZone::White; }
    if( RfZoneLimits::MaxRfRange > distance and distance >= RfZoneLimits::Alarm      )
    { return RfZone::Red; }
    if( RfZoneLimits::Alarm      > distance and distance >= RfZoneLimits::Warning    )
    { return RfZone::Orange; }
    if( RfZoneLimits::Warning    > distance and distance >= RfZoneLimits::Caution    )
    { return RfZone::Yellow; }
    if( RfZoneLimits::Caution    > distance and distance >= 0 )
    { return RfZone::Green; }
    // else
    return RfZone::White;
  }
  
  //@}
  
  /** \name COM Graph */ //@{
  
  /** \brief The COM graph of this Node.
   * \details The COM graph represents the current COM network of the ownership,
   * i.e. it represent the communication datalinks.
   */
  ComGraph::Graph_t comGraph;
  
  ComGraph::VertexNameMap_t       comGraphNameMap;
  ComGraph::VertexIdMap_t         comGraphIdMap;
  ComGraph::VertexPosition2dMap_t comGraphPositionMap;

  /** \brief Update the COM graph after a recent change of a tracked node.
   * 
   * \pre Normally a call to this function should be preceeded by a call to
   *  updateRfPoly(nodeId)!
   * 
   * The process to determine connectivity is a multi step process:
   * 
   * 1. Update the COM graph edges:
   * - Disconnect the dynamic node (dynamicNodeId) from the network (i.e. erase
   *   all its edges in the comGraph).
   * - Check if any of the already connected nodes' positions are inside
   *   the dynamic node's freshly updated R/F polygon. (Hence the requirement
   *   to have called updateRfPoly(nodeId) before calling this function.)
   * - if yes, create an appropriate edge
   * 
   * 2. Determine Connectedness:
   * - Update the connected components of all Nodes in the COM graph.
   * - Set the connectedness of all Nodes in the ownship's connected componend 
   *   to TRUE, all others to FALSE;
   * 
   * 
   * \todo Split this function into a const version that requires all the 
   *  arguments to be passed in (i.e. NodeID, ComGraph, rfPolygon) and then call 
   *  that function from some convenience overloads (i.e. with the signature the
   *  current function has.)
   * 
   * \param[in] dyanmicNodeId Changes related to this node are checked and 
   *  incorporated. (Other nodes are treated as/assumed to be constant.)
   * \return TRUE if the node in question is (potentially multi-hop) connected 
   *  to this node; FALSE otherwise.
   */
  bool updateComGraph( NodeId const & dynamicId );  
  
#ifdef WITH_GRAPHICS  
  
  /** \brief The graphics representing the COM graph. */
  QGraphicsItemGroup* comGraphGraphics = nullptr;
  
  /** \brief Update the graphics representing the COM graph.
   * \details This function \e completely erases and redraws the collection of 
   * lines which represent the COM graph.
   * \note Only available WITH_GRAPHICS 
   */
  void updateComGraphGraphics();
#endif //WITH_GRAPHICS
  
  //@}
  
  /** \name Voronoi (Vroni) Graph */ //@{
  
  /** \brief The full weighted medial axis graph (i.e. "Voronoi diagram") */
  VroniGraph::Graph vroniGraph;
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
  /** \brief The graphics representing the VRONI graph. */
  QGraphicsItemGroup* vroniGraphGraphics = nullptr; 
  
  /** \brief The graphics representing the VRONI site-to-shape associations. */
  QGraphicsItemGroup* vroniSiteShapeGraphics = nullptr;
  /** \brief The graphics representing the VRONI edge-to-site associations. */
  QGraphicsItemGroup* vroniEdgeSiteGraphics = nullptr;
  /** \brief The graphics representing the VRONI edge-to-shape associations. */
  QGraphicsItemGroup* vroniEdgeShapeGraphics = nullptr;
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
  //@}  
  
  /** \name Route Graph */ //@{
      
  /** \brief The route graph of this Node.
   * \details This subgraph of the vroniGraph is does not have any non-cyclic 
   * branches and as such makes up a no-dead-end path network usable for 
   * traversing the 2D environment.
   * \sa routeGraphIsCurrent */
  RouteGraph::Graph routeGraph;
  
  /** \brief A flag to keep track of the status of the routeGraph.
   * \sa updateRouteGraph()  */
  bool routeGraphIsCurrent = false;

//   auto routeGraphLocationMap;
  
  /** \brief Update the route graph and related graphics.
   * \details This function computes a fully cyclic route graph based on the 
   * Voronoi diagramm for the current motionBoundary and osbstacleData, i.e. 
   * the state of the arena polygon does \e not matter. 
   * Executing this funciton sets routeGraphIsCurrent to TRUE.
   * 
   * The function also updates vroniGraph. */
  void updateRouteGraph();
  
#ifdef WITH_GRAPHICS
  /** \brief The graphics representing the Route graph. */
  Graphics_2D::GraphItem* routeGraphGraphics = nullptr;
#endif // WITH_GRAPHICS
  //@}
   
  /** \brief Update the graphs derived from the routeGraph.
   * \details This updates the obstacleGraph and the bifurcationGraph and sets 
   * the corresponding isCurrent flags to TRUE.
   */
  void updateDerivedGraphs();
   
  /** \brief A mutex used by updateDerivedGraphs() . */
  QMutex derivedGraphUpdateMutex;

  
  /** \name Obstacle Graph */ //@{
   
  /** \brief The obstacle graph of this Node.
   * \details Obstacles are connected by an edge in the obstacle graph if they
   * are neighbors, i.e. if their respective obstacle cells share a common 
   * edge. As such, the obstacle graph is the direct dual to the bifurcation 
   * graph.
   * \sa updateDerivedGraphs()
   * \sa obstacleGraphIsCurrent 
   * \sa bifurcationGraph
   */
  ObstacleGraph::Graph obstacleGraph;
  
  /** \brief A flag to keep track of the status of the obstacleGraph.
   * \sa updateDerivedGraphs()  */
  bool obstacleGraphIsCurrent = false;
  
  // see http://www.boost.org/doc/libs/1_54_0/libs/property_map/doc/iterator_property_map.html
  // TODO: T and R need to be defined!!! 
  using ObstacleDataMap = boost::iterator_property_map<
      std::vector<Obstacle>::iterator, /// Iterator
      boost::property_map<ObstacleGraph::Graph,boost::vertex_index_t>::type, /// OffsetMap
      Obstacle, /// The value type of the iterator.
      Obstacle& /// The reference type of the iterator.
    >;
    
#ifdef WITH_GRAPHICS
  /** \brief The graphics representing the Obstacle graph. */
  QGraphicsItemGroup* obstacleGraphGraphics = nullptr;

  #if defined DEBUG_GRAPHICS
  /** \brief The graphics representing the association of Obstacles to 
   * bifurcation graph edges. */
  QGraphicsItemGroup* obstacleBiEdgeAssocGraphics = nullptr;
  
  /** \brief The graphics representing the association of Obstacles to 
   * route graph vertices. */
  QGraphicsItemGroup* obstacleRouteVertexAssocGraphics = nullptr;
#endif // DEBUG_GRAPHICS
  
#endif // WITH_GRAPHICS
  
  /** \brief Get a list of ObstacleCells which are (partially) inside the outline.
   * \note This checks \e all known obstacle cells, i.e. no smarts are employed!
   * \todo Implement a smart version that starts at a hinted cell and then makes
   * use of a breadth first search through the obstacle graph to quickly find
   * all overlapping cells via this "flooding" approach.
   */
  std::set<ElementId> getObstacleCellsWithinArea(Geometry::Polygon_2D::ring_type const& outline);
  
  /** \brief Get a list of Obtacles which are (partially) inside the outline. */  
  std::set<ElementId> getObstaclesWithinArea(Geometry::Polygon_2D::ring_type const& outline);
  
  //@}
    
  /** \name Face Graph
   * \details This is a DEBUG section which shouldn't need toshow up in the 
   * release code. */ //@{
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
  /** \brief The graphics representing the raw face graph.*/
  QGraphicsItemGroup* faceGraphGraphics = nullptr;
  
  /** \brief The graphics representing the raw face graph.*/
  QGraphicsItemGroup* faceToObstacleCorrespondence = nullptr;
#endif // WITH_GRAPHICS&& DEBUG_GRAPHICS
  
  //@}
    
  /** \name Bifurcation Graph */ //@{
  
  /** \brief The graph representing actual route intersections on the routeGraph.
   * \details The bifurcation graph is an abstraction of the route graph. 
   * Vertices of the bifurcation graph correspond to route graph vertices with a 
   * degree of (at least) three, i.e. vertices where the route graph splits or
   * branches. (Due to the nature of the route graph, degree 4 vertices, i.e. 
   * perfect 4-way intersections, are extremely rare -- unless the environment 
   * is specifically crafted to create such a situtaion.)
   * As such, the bifurcation graph can/could be used to find routes through
   * the environemnt as the route length, encoded in the \c edge_weight property
   * is identical to the length of the route graph edges connecting the two
   * interesction vertices. 
   * 
   * The bifurcation graph is the dual of the obstacle graph.
   * 
   * \todo The bifurcation graph should allow a much faster computation of 
   * paths through a complex environement as it greatly reduces the number of
   * vertices (a major driver in computational complexity). A first step there
   * would be to augment the bifurcation graph with the start and end points of
   * the search as those most likely won't correspond to intersections, i.e. 
   * bifurcation graph vertices. Once the path has been computed, a "higher 
   * resolution" set of waypoints can be obtained by remapping the bifurcation
   * edges to the underlying route graph ones.
   * (This seems to be a very good bridge to any type of conventional vehiclura
   * navigation based on digital maps...)
   * 
   * \sa updateDerivedGraphs() 
   * \sa obstacleGraph
   */
  BifurcationGraph::Graph bifurcationGraph;  
  
//   /** \brief Mapping bifurcation graph vertices to the corresponding route graph 
//    * vertices. */
//   std::unordered_map<Simple2dGraph::Vertex_Descriptor, RouteGraph::Vertex_Descriptor> biVertexToRouteVertexMap;
  
  /** \brief The "database" holding the Locations associated with each 
   * bifurcation graph edge.
   * \note The locations associated with the source and target vertices of the
   * edges are \e not part of this dataset. */
  std::map<BifurcationGraph::Edge_Descriptor, std::set<LocationId>> biEdgeLocationData;
  
  
  /** \brief Reduce the current routeGraph to its intersection vertices and store that
   * as the bifurcationGraph.
   *\details This initially copies the routeGraph into the the bifurcationGraph,
   * which invalidates all vertex and edge descriptors, and then eliminates all 
   * vertices in the bifurcation graph that have a degree of 2. */
  void updateBifurcationGraph();
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS  
  void updateBifurcationGraphGraphics();  
  QGraphicsItemGroup* bifurcationGraphGraphics = nullptr;
  QGraphicsItemGroup* biEdgeAssociationGraphics = nullptr;
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
  //@}  
     

  /** \name R/F Graph */ //@{
  
  
  /** \brief R/F connectability graph.
   * \details The rfGraph is a range limited visibilty graph of various 
   * locations, primarily of the vertices of the routeGraph, and as such 
   * represents possible COM connectedness of those locations. 
   * 
   * \internal As these vertices represent non-moving locations (identifyable 
   * through a LocationId), these vertices are referred to as \e static vertices.
   * 
   * The rfGraph normally also includes at least one vertex of a node, the 
   * ownership. 
   * 
   * \internal As these verticese represent moving locations (identifyable 
   * through a NodeID), these vertices are referred to as \e dynamic vertices.
   * 
   * \note The ownship vertex has a NULL_LOCATION_ID and has the index zero.
   */
  RfGraph::Graph rfGraph;
  
  /** \brief A reverse lookup from LocationId to RfGraph::Vertex_Descriptor. */
  std::unordered_map<LocationId, RfGraph::Vertex_Descriptor> locationId_to_rfVertex;
  
  /** \brief Validity of the rfGraph.
   * \details TRUE if the rfGraph corresponds to the current target environemnt,
   * FALSE otherwise. */
  bool isRfGraphValid;
  
  /** \brief The R/F graph vertex representing a Blip in the R/F graph.
   * \details This vertex is used to capture the "source" when computing COM 
   * formations. */
  std::unordered_map<NodeId, RfGraph::Vertex_Descriptor> dynamicRfVertex; 
  
#if defined WITH_GRAPHICS && defined DEBUG_GRAPHICS
  /** \brief ThegGraphics representing the R/F graph. */
  Graphics_2D::GraphItem* rfGraphGraphics = nullptr;
#endif // WITH_GRAPHICS && DEBUG_GRAPHICS
  
  /** \brief Compute the R/F graph for all locations and all tracked Blips in 
   * the environement.
   * \sa dynamicRfVertex
   * \pre The location of the dynamicRfVertex is covered by the environment.
   * \param[in] environement The region for which to compute the R/F graph.
   */
  void computeRfGraph(Geometry::Polygon_2D const& environment);
  
  /** \brief Recompute the R/F connectivity for the dynamic vertex and update
   * the graph. */
  void updateRfGraph(){ 
    qCritical("%s: Not yet implemented.",this->q_func()->qualifiedName().toStdString().c_str());
  }; //TBD
  
  //@}
  
  
  /** \name Formation Graph */ //@{
  
  FormationGraph::Graph formationGraph;
  
  bool isFormationGraphValid = false;
  
#ifdef WITH_GRAPHICS
  /** \brief The formation graph, i.e. the combination of ingress and loop. */
  Graphics_2D::GraphItem* formationGraphGraphics = nullptr;

  /** \brief The ingress paths to the target cell intersection vertices. */
  mutable QGraphicsItemGroup* ingressGraphics = nullptr;
  
  /** \brief The COM loops around the target obstacle. */
//   mutable QGraphicsItemGroup* comLoopGraphGraphics = nullptr;
  mutable Graphics_2D::GraphItem* comLoopGraphGraphics = nullptr;
#endif // WITH_GRAPHICS
  
//NOTE: this is currently done in proposeComFormation/processComFormationProposal  
//   /** \brief */
//   void computeFormationGraph(Geometry::Polygon_2D const& environment);
 
  
  //@}

  
  
  /** \name Formations 
   * \details Formations are groups of assignments of Nodes to positions in 
   * space. */ //@{
  
  /** \brief  The individual pairing of a Node to a position. */ 
  class FormationPairing : public std::pair<NodeId, Geometry::Point_2D> 
  {
  public:
    FormationPairing(NodeId id, Geometry::Point_2D position)
    :std::pair<NodeId, Geometry::Point_2D>(id, position){};
    NodeId nodeId() const
    { return first; }
    void setNodeId(NodeId id)
    { first = id; };
    Geometry::Point_2D position() const
    { return second; }
    void setPosition(Geometry::Point_2D position)
    { second = position; };
  };
  
  /** \brief A collections of Node-to-position pairings, forming a formation.
   * \todo More work needs to be done to work this out: I would assume that 
   * formations are normally given in relative locations, i.e. the positions
   * of the involved nodes are defined relative to each other and then the 
   * complete Formation is given a 'global' position that could be used as an
   * origin for the other 'relative' positions...
   * \todo Ok, so maybe packing this in some sort of graph structure would make 
   * sense: vertices obviously represent positions and edges defined distances
   * Oh heck, thinking back that rings a bell about all kinds of stuff: 
   * graph stiffness, etc. pp. ... [here be dragons]
   */
  using Formation = std::vector<FormationPairing>;
    
  //@}
  
  
  /** \name Topics */ //@{

  /** \brief The topic for exchanging State messages.  */
  BroadcastTopic<IDL::Messages::State,Node,TopicOptions::SendReceive>* stateTopic = nullptr;

  /** \brief A pointer to the struct used to send State messages.
   * 
   * The message data is current, i.e. it always reflects the most up-to-date 
   * information available.
   * 
   * \internal 
   * The stateMessage is essentially carrying the state information of the
   * ownship, which is stored in the ownshipBlip. As such, the stateMessage
   * is automatically updated whenever the data stored in the ownshipBlip is
   * altered. This is accomplished via reimplemented set-functions in 
   * OwnshipBlip.
   * 
   */
  IDL::Messages::State* stateMessage = nullptr;
  
  /** \brief The topic for exchanging State messages. */
  BroadcastTopic<IDL::Messages::Status,Node,TopicOptions::SendReceive>* statusTopic = nullptr;

  /** \brief A pointer to the struct used to send State messages. 
   * 
   * \note The message data is not automaticall updated, i.e. it might contain 
   * historical data!
   * 
   * \sa updateStatusMessage();
   */
  IDL::Messages::Status* statusMessage = nullptr;
  
  /** \brief Update the data stored in the statusMessage. 
   * 
   * Gets the current stateMachine configuration, compares that to the states
   * which are to be reported, and stores the matching ones in the message.
   * 
   * \sa reportedStates
   */ 
  void updateStatusMessage();
  
  /** \brief The topic for exchanging State messages. */
  BroadcastTopic<IDL::Messages::Datum,Node,TopicOptions::SendReceive>* datumTopic = nullptr;

  /** \brief A pointer to the struct used to send State messages. */
  IDL::Messages::Datum* datumMessage = nullptr;
  
  /** \brief Update the data stored in the datumMessage. */ 
  void updateDatumMessage();
  
  /** \brief The topic for exchanging Flightplan messages. */
  BroadcastTopic<IDL::Messages::Flightplan,Node,TopicOptions::SendReceive>* flightplanTopic = nullptr;
  
  IDL::Messages::Flightplan* flightplanMessage = nullptr;
  
  //@}
     
  
  
  
  
  /** \brief A list containing the states which the node should report.
   * 
   * The StateMachine that each node has (from the underlying SimItem) has a
   * multitude of states, each (well, most) of which have an associated
   * IDL::Data::StateDescriptorId. 
   * 
   * When Nodes report about the state of their StateMachine, a.k.a. their 
   * \e mode, not all of those states are of interest.
   * This list containes the StateDescriptorIds this Node will report via
   * the statusTopic. It is initilized in the NodePrivate constructor.
   * 
   * \internal The (debug) variable honorReportedStatesSubSet in 
   * NodePrivate::updateStatusMessage() eventually determines whether this 
   * set of states is honored when selecting what to report.
   */
  static QList<IDL::Data::StateDescriptorId> reportedStates;
  
    
  /** \brief Get a list of Blips which are in a certain mode. 
   *\param[in] mode The mode a Blip must be in to be included into the list
   *\return A list of all (known) Blips in the requested mode. */  
  std::list<Blip*> getBlipsByMode(IDL::Data::StateDescriptorId mode) const;  
   
  /** \brief Check whether a blipData with the given NodeId is present. 
   * \param[in] id The NodeId to locate in blipData.
   * \return TRUE if blip(id) is valid; FALSE otherwise.
   */
  bool blipExists(NodeId const & id) const;
  
  /** \brief Count the number of known Blips in the given state. */
  unsigned int blipCount(IDL::Data::StateDescriptorId const & state) const;
  
  
  void setOwnshipStateVector( const AMG::Vector& position
                             ,const AMG::Vector& velocity
                             ,const AMG::EulerAngleTupel& attitude );
  
  /** \brief Find the Blips that are COM-connected to a target Node if other 
   * Nodes are no longer part of the Network.
   * \details This function creates a copy of the current comGraph, disconnects
   * the exluded Nodes and then computes the connected components. All Blips in the 
   * same component as the target Blip will be returned. 
   * \param[in] target The Node that defines the connected zone, i.e. the 
   * "center".
   * \param[in] excludes The Nodes which will be taken out of the COM network.
   * \return A list of Blips that are in the connected componend of the target.
   */
  QList<NodeId> comRelayNodes(NodeId const& target, QList<NodeId> const& excludes) const;
  
  /** \brief Find the Blips that are COM-connected to a target Node if another 
   * Node is no longer part of the Network.
   * \overload
   * \details This function creates a copy of the current comGraph, disconnects
   * the exluded Node and then computes the connected components. All Blips in the 
   * same component as the target Blip will be returned.
   * \param[in] target The Node that defines the connected zone, i.e. the 
   * "center".
   * \param[in] exclude The Node which will be taken out of the COM network.
   * \return A list of Blips that are in the connected componend of the target.
   */
  QList<NodeId> comRelayNodes(NodeId const& target, NodeId const& exclude) const
  {
    QList<NodeId> list = {exclude};
    return comRelayNodes(target,list);
  }
  
  /** \brief Find the Blips that are COM-connected to a target Node if the 
   * ownship Node is taken out of the Network
   * \overload
   * \details This function creates a copy of the current comGraph, disconnects
   * the ownship Node and then computes the connected components. All Blips in the 
   * same component as the target Blip will be returned.
   * \param[in] target The Node that defines the connected zone, i.e. the 
   * "center".
   * \return A list of Blips that are in the connected componend of the target.
   */
  QList<NodeId> comRelayNodes(NodeId const& target) const
  { return comRelayNodes(target, ownshipId); };
  
    
  /** \brief Update the rfRelayFlag property of Blips.
   * \internal There is no real need to do this at the Node level, which is why 
   * this is an empty function. However, at this point all Node descendents
   * (i.e. Vehilce and ControlStation) do this and hence providing the function,
   * as well as the related updated calls here seems to make sense.
   * \todo Maybe this could be made smarter by using com graph properties like 
   * being biconnected? Boost also provides the articulation points for a bi-
   * connected graph, i.e. points which when removed destroy the connectedness
   * See http://www.boost.org/doc/libs/1_55_0/libs/graph/doc/biconnected_components.html
   */
  virtual void updateRfConnectedArea(){};
  
  
  
  /** \brief Initialize the Node specific states of the statemachine. */
  void initializeStates();
  
  /** \brief Get the initial position specified in the XML settings. */
  AMG::Vector getXmlInitialPosition();
  
  void initializePosition();
  
  void initializeDatalink();
  
  /** 
   * This function replicates the Blip cration process normally tied to the 
   * detection of a new participant. However, as the participant of the ownship
   * doesn't detect itself, this manual initilization is necessary.
   * Furthermore, the source of some of the data stored in the OwnshipBlip 
   * is different: for other Blips data has to come via the datalink, for the
   * ownshipBlip, this can't be the case.
   */
  void initializeOwnshipBlip();
  
  void initializeComGraph();
  
  void initializeRouteGraph();
  
  void initializeDerivedGraphs();
  
#ifdef WITH_GRAPHICS
  /** \brief Initialize the generic graphics item, i.e. the non-blip elements.
   * \note Only available WITH_GRAPHICS  */
  void initializeGraphics();
    
  /** \brief Initialize the model/view-models used in a Node.
   * \note Only available WITH_GRAPHICS */
  void initializeModels();
#endif // WITH_GRAPHICS
  
  void unInitializeNode();
  
  
  /** \brief Create a ''random'' number somewhere in (0,1) */
  double rZeroOne() const;
  
  /** \brief Create a ''random'' number somwhere in (-1,1). */
  double rMinusOneOne() const;
  
  /** \brief A debug function shifting the position of the Node randomly. 
   * \deprecated
   * \param[in] distance The maximum +/- position offset. */
  void randomWalk(double const & distance = 0.5);
  
  
  
  
  
  
  
  
  /** \brief Create and add a new blip from the given data.
   * 
   * \note Making this function virtual isn't really that optimal as the 
   * NodePrivate version \e needs to be called in order to call \c new. As such,
   * it is anticipated that core used know what they are doing and periphery 
   * users listen to the Node::blipAdded() signal.
   * 
   * \param[in] id The Id of the new Blip.
   * \param[in] type The type of the new Blip.
   * \param[in] name Tha name of the new Blip.
   * \param[in] propertyList A list of properties the Blip gets assigned.
   * \param[in] blip (optional) A pointer to a specific Blip instance which 
   * should be used to hold the new blip. If the input is nullptr a generic Blip
   * will be used.
   */
  virtual void initializeBlip( const NodeId& id, const Node::Type& type
  , QString const & name, QStringList const & propertyList, Blip* blip = nullptr );
  
  /** \brief Update tertiary blip data after a change in primary blip data. */
  void updateTertiaryBlipData(Blip* const blip);
  
  /** \brief Delete the blip for the given id.
   * \details Locks the blipDeletionMutex while running. */
  virtual void deleteBlip(NodeId const &id);
    
  /** \brief A mutex used by deleteBlip() . */
  QMutex blipDeletionMutex;
  
  
  
  
  
  /** \brief Get the index of the obstacle closest ot the given location.
   * \details Searches the obstacle cell geometries to find the one that 
   * contains location. If a hint is given, starts at that cell, then searches
   * the neighboring ones. Falls back to a global search if location isn't
   * found in any of the neighbors.
   * \todo Make this a breadth first search on the obstcle graph.
   * \param[in] location The location the obstacle should be the closest one to.
   * \param[in] hint (optional) The ID of the obstacle to start the search at.
   * \return The ID of the obstacle closest to location. Returns 
   * NULL_ELEMENT_ID if no closest obstacle could be determined.
   */
  ElementId getClosestObstacle(const Geometry::Point_2D& location, const ElementId& hint = NULL_ELEMENT_ID);
  
  /** \brief Get the closest point on the route graph for the indicated Blip. */
  Geometry::Point_2D closestPointOnRouteGraph(Blip const * const blip);
  Geometry::Point_2D closestPointOnRouteGraph(Geometry::Point_2D const& fromHere,
                                              ElementId const& closestObstacleHint = NULL_ELEMENT_ID
                                              );
  
  
  
  
  
  
  
  
  
  
  /** \brief Compute the \Delta-disc geometry for the given parameters.
   * \param[in] center The coordinates of the center of the disc
   * \param[in] range The radius of the disc
   * \return A 100 corner ring geometry approximating the disc.
   */
  Geometry::Polygon_2D::ring_type deltaDisc(
    const Geometry::Point_2D& center,
    const boost::geometry::coordinate_type<Geometry::Polygon_2D>::type& range
  ) const;

    /** \brief Compute a pie-shaped sector of a \Delta-disc.
   * 
   * \param[in] center The coordinates of the root of the \Delta-wedge.
   * \param[in] range The radius of the underlying \Delta-disc in [m]
   * \param[in] direction The "heading" of the axis of symmetry of the wedge in
   *  [degrees].
   * \param[in] openingAngle The total opening angle of the wedge in [degrees]. 
   *  (Radial boundaries are at symmetry axis +/- 0.5 * openingAngle.)
   * \return The intersection of a deltaDisc and a symmetrical pie wedge with 
   *  the given orientation and opening angle.
   */
  Geometry::Polygon_2D::ring_type deltaPie(Geometry::Point_2D const & center
    ,const boost::geometry::coordinate_type<Geometry::Polygon_2D>::type& range
    ,double const & direction, double const & openingAngle
  ) const;
  
  
  
  
  
  /** \brief Compute a \Delta-polygon centered at location with given range.
   * 
   * This function takes the intersection of the environment2D and a \Delta-disc
   * with radius range centered at location and returns it as a \Delta-polygon.
   * 
   * \param[in] location The coordinates of the center of the \Delta-disc. 
   * \param[in] range The radius of the \Delta-disc in [m]
   * \return The intersection of a 100 corner circle and the current 
   *  environment2D.
   */
  Geometry::Polygon_2D deltaPolygon( const Geometry::Point_2D& location,
  const boost::geometry::coordinate_type<Geometry::Polygon_2D>::type& range) const
  { return subEnvironment(location, deltaDisc(location,range)); };
  
  /** \brief Compute a \Delta-polygon pie centered at location with given range.
   * 
   * This function takes the intersection of the environment2D and a \Delta-pie 
   * with radius range centered at location, headring direction, opening angle
   * openingAngle. and returns it as a \Delta-polygon.
   *
   * \sa deltaPie()
   * 
   * \param[in] location The coordinates of the center of the \Delta-disc. 
   * \param[in] range The radius of the \Delta-disc in [m]
   * \param[in] direction The "heading" of the axis of symmetry of the pie wedge
   *  in [degrees].
   * \param[in] openingAngle The total opening angle of the pie wedge in 
   *  [degrees]. (Radial boundaries are at symmetry axis +/- 0.5 * openingAngle.)
   * \return The intersection of the current environment2D and a deltaPie.
   */
  Geometry::Polygon_2D deltaPolygon( const Geometry::Point_2D& location
    ,const boost::geometry::coordinate_type<Geometry::Polygon_2D>::type& range
    ,double const & direction, double const & openingAngle
  ) const
  { return subEnvironment(location, deltaPie(location,range,direction,openingAngle)); };
    
  
  
  /** \brief Intersects the enviroment with another polygon.
   * 
   * This function intersects the given \c stencil polygon with the current 
   * environment. In case of disconnected sets, it returns the polygon that 
   * covers the described \e location.
   * This is the core part of the work done in deltaPie and deltaPolygon.
   * 
   * \param[in] location The location which needs to be in the returned polygon.
   * \param[in] stencil The polygon intersected with the enviroment.
   * \return A subset of the environment containg location. */
  Geometry::Polygon_2D subEnvironment( Geometry::Point_2D const& location,
    Geometry::Polygon_2D::ring_type const& stencil) const;
  
  /** \brief Update the Visibility polygon to match the current position.
   * \overload
   * \deprecated
   *
   * \param[in] id The id of the blip for which to compute the visibility 
   *  polygon.
   * \param[in] minDistance (optional) The minimum distance the blip needs to
   *  have moved before a new R/F polygon is computed.
   * \return TRUE, if a new visibility polygon has been computed; FALSE 
   *  otherwise.
   */
  bool updateVisPoly(NodeId const & id,
                     double const & minDistance  = 2.0); //FIXME: Magic Numbers
  
  /** \brief Update the Visibility polygon to match the current position.
   * This function computes the non-range-limited visibility polygon for the
   * indicated node, based on the corresponding record in blipData.
   *
   * \param[in] blip The blip for which to compute the visibility 
   *  polygon.
   * \param[in] minDistance (optional) The minimum distance the blip needs to
   *  have moved before a new R/F polygon is computed.
   * \return TRUE, if a new visibility polygon has been computed; FALSE 
   *  otherwise.
   */
  bool updateVisPoly(Blip* const blip,
                     double const & minDistance  = 2.0); //FIXME: Magic Numbers
  
  
  /** \brief Compute the R/F polygon for the given position.
   * \param[in] position The center of the polygon (i.e. the "guard location")
   * \param[out] rfRing The ring holding the R/F polygon. 
   * \return TRUE if the computation was successfull; FALSE otherwise;
   */
  bool computeRfPolygon( Geometry::Point_2D const & position, Geometry::Polygon_2D::ring_type & rfRing );
  
  
  /** \brief Update the R/F polygon for the indicated node.
   * 
   * This function updates the R/F polygon for the indicated node based on the
   * corresponding blipData. If the R/F polygon has successfully been updated,
   * the function returns TRUE.
   * 
   * \param[in] blip The blip for which to compute the R/F polygon.
   * \param[in] minDistance (optional) The minimum distance the blip needs to
   *  have moved before a new R/F polygon is computed.
   * \return TRUE, if a new R/F polygon has been computed; FALSE otherwise.
   */
  bool updateRfPoly(Blip* const blip,
                    double const & minDistance = 2.0 ); //FIXME: Magic Numbers
  
  
  /** \brief Update the Field-of-View (FOV) polygon for the indicated node.
   * 
   * Recompute the FOV polygon for an electro-optical sensor, aka a camera,
   * that is mounted at the nose of the vehicle. The FOV polygon should 
   * represent what this camera should/could see.
   * 
   * In order to preserve computation, the function only recomputes the FOV 
   * polygon if either the minimum distance threshold or the minimum yawing 
   * angle threshold has been reached.
   * 
   * \param[in] blip The blip for which to compute the FOV polygon.
   * \param[in] minDistance (optional) The minimum distance the blip needs to
   *  have moved before a new FOV polygon is computed.
   * \param[in] minRotation_deg (optional) The minimum angle the blip needs to 
   *  have yawed before a new FOV polygon is computed.
   * \return TRUE, if a new FOVF polygon has been computed; FALSE otherwise.
   */
  bool updateFovPoly(Blip* const & blip,
                     double const & minDistance = 2.0, //FIXME: Magic Numbers
                     double const & minYaw_deg = 2.0 );//FIXME: Magic Numbers
  
  
    
  /** \brief Post the state machine event associated with the given command. */
  void postCommand(IDL::Commands::CommandTypes const & command);

//   /** \brief Get the degree-3 nodes of the routeGraph surrounding the target. */
//   std::list<RouteGraph::Vertex_Descriptor> getBranchNodes( ElementId const & targetId );
  
  
  /** \name Dedicated Listeners for built-in topics */ //@{
  
  /** \brief A listener for the Publication Builtin-TopicData DataReader.
   * 
   * This listener is attached to a built-in data reader that comes with the 
   * built-in subscriber attached to any participant.
   * 
   * For more details, see http://community.rti.com/content/forum-topic/detect-presence-domainparticipants-datawriters-and-datareaders-dds-domain
   * 
   */
  class BuiltInTopic_Publications_Listener : public ValidatingDataReaderListener
    < DDSPublicationBuiltinTopicDataDataReader
    , DDS_PublicationBuiltinTopicDataSeq
    , DDS_PublicationBuiltinTopicData
    , Node >
  {
  public:
    using Base = ValidatingDataReaderListener
    < DDSPublicationBuiltinTopicDataDataReader
    , DDS_PublicationBuiltinTopicDataSeq
    , DDS_PublicationBuiltinTopicData
    , Node >;

    /** \brief A status mask capturing all implemented callback routines. */
    static const DDS_StatusMask implementedCallbacks = Base::implementedCallbacks ;
      
  public:
    BuiltInTopic_Publications_Listener(Node* parent)
      :ValidatingDataReaderListener(parent){};

    
  protected:
    virtual void on_valid_data_available ( DDS_PublicationBuiltinTopicData& data,
                                           DDS_SampleInfo& info,
                                           DDSPublicationBuiltinTopicDataDataReader* reader )
    {
      NodeId id = NodeId(data.participant_key);
      if(  parent()->blipExists(id) )
      {
        
        qDebug() << parent()->objectName() << ": Publication  detected:"
        << parent()->blip(NodeId(data.participant_key)).name() 
        << "publishes topic" << QString(data.topic_name);
//         << "with type" << QString(data.type_name)
//         << "( .key =" << data.key << ")";;
      }
      else //The blip might not yet be registered at this point!
      {     
        qDebug() << parent()->objectName() << ": Publication  detected:"
        << data.participant_key
        << "publishes topic" << QString(data.topic_name);
//         << "with type" << QString(data.type_name)
//         << "( .key =" << data.key << ")";
      }
    };
  
    virtual void on_sample_info_available ( DDS_SampleInfo& info, DDSPublicationBuiltinTopicDataDataReader* reader )
    {};
  };
  
  /** \brief A listener for the Subscription Builtin-TopicData DataReader.
   * 
   * This listener is attached to a built-in data reader that comes with the 
   * built-in subscriber attached to any participant.
   * 
   * For more details, see http://community.rti.com/content/forum-topic/detect-presence-domainparticipants-datawriters-and-datareaders-dds-domain
   */
  class BuiltInTopic_Subscriptions_Listener : public ValidatingDataReaderListener
    < DDSSubscriptionBuiltinTopicDataDataReader
    , DDS_SubscriptionBuiltinTopicDataSeq
    , DDS_SubscriptionBuiltinTopicData
    , Node >
  {
  public:
    using Base = ValidatingDataReaderListener
    < DDSSubscriptionBuiltinTopicDataDataReader
    , DDS_SubscriptionBuiltinTopicDataSeq
    , DDS_SubscriptionBuiltinTopicData
    , Node >;
      
    /** \brief A status mask capturing all implemented callback routines. */
    static const DDS_StatusMask implementedCallbacks = Base::implementedCallbacks ;
    
  public:
    BuiltInTopic_Subscriptions_Listener(Node* parent)
      :ValidatingDataReaderListener(parent){};
      

  protected:
    virtual void on_valid_data_available ( DDS_SubscriptionBuiltinTopicData& data,
                                            DDS_SampleInfo& info,
                                            DDSSubscriptionBuiltinTopicDataDataReader* reader )
    {
      NodeId id = NodeId(data.participant_key);
      if( parent()->blipExists(id) )
      {
        qDebug() << parent()->objectName() << ": Subscription detected:"
          << parent()->blip(id).name()   
          << "subscribed to topic" << QString(data.topic_name) ;
    //         << "with type" << QString(data.type_name)
    //         << "( .key =" << data.key << ")";
      }
      else //The blip might not yet be registered at this point!
      {     
        qDebug() << parent()->objectName() << ": Subscription detected:"
          << data.participant_key
          << "subscribed to topic" << QString(data.topic_name) ;
    //         << "with type" << QString(data.type_name)
    //         << "( .key =" << data.key << ")";
      }
    };
  
    virtual void on_sample_info_available ( DDS_SampleInfo& info, DDSPublicationBuiltinTopicDataDataReader* reader )
    {};
  };

  /** \brief A listener for the Participant Builtin-TopicData DataReader.
   * 
   * This listener is attached to a built-in data reader that comes with the 
   * built-in subscriber attached to any participant.
   * 
   * For more details, see http://community.rti.com/content/forum-topic/detect-presence-domainparticipants-datawriters-and-datareaders-dds-domain
   * 
   */
  class BuiltInTopic_Participant_Listener : public ValidatingDataReaderListener
    < DDSParticipantBuiltinTopicDataDataReader 
    , DDS_ParticipantBuiltinTopicDataSeq
    , DDS_ParticipantBuiltinTopicData
    , Node >
  { 
  public:
    using Base = ValidatingDataReaderListener 
    < DDSParticipantBuiltinTopicDataDataReader 
    , DDS_ParticipantBuiltinTopicDataSeq
    , DDS_ParticipantBuiltinTopicData
    , Node >; 
    
    /** \brief A status mask capturing all implemented callback routines. */
    static const DDS_StatusMask implementedCallbacks = Base::implementedCallbacks
      | DDS_LIVELINESS_CHANGED_STATUS 
      ;      

  public:
    BuiltInTopic_Participant_Listener(Node* parent)
      :ValidatingDataReaderListener(parent){};
      
    virtual void on_liveliness_changed( DDSDataReader* reader,
      DDS_LivelinessChangedStatus const & status)
    {  
      qDebug() << parent()->qualifiedName() << "(BuiltInTopic_Participant_Listener)"
        << ": DataReader liveliness changed for topic" << reader->get_topicdescription()->get_name() << ".\n"
        << "    alive count:" << status.alive_count << "(change:"<< status.alive_count_change << ")\n"
        << "not-alive count:" << status.not_alive_count << "(change:" << status.alive_count_change << ")";
//         << "last publication handle :" << status.last_publication_handle << "\n";
      
      { /// remove the stale participant
        
        auto m_parent_node_private = parent()->d_func();
        DDS_InstanceHandle_t staleParticipantHandle = status.last_publication_handle; //FIXME NOTE This is most likely _NOT_ the participant handle...
        
        if( m_parent_node_private->detectedParticipants.contains( staleParticipantHandle ) )
        {
          auto staleParticipantId = m_parent_node_private->detectedParticipants.value( staleParticipantHandle );
         
          qDebug() << parent()->qualifiedName() << "(BuiltInTopic_Participant_Listener)"
            << ": Removing a stale DDS participant. (" << parent()->blip(staleParticipantId).name() <<")";
         
          m_parent_node_private->deleteBlip( staleParticipantId );
          m_parent_node_private->detectedParticipants.remove( staleParticipantHandle );
        }
      }
    };

      
  protected:
    
    /** \brief React to the detection of a previoulsy unknown participant.
     * 
     * This is inspired by a piece of Java code that is available on
     * the RTI FileExchange at http://community.rti.com/filedepot?cid=6&fid=18
     * 
     * The corresponding snippet looks like this:
\code{.java}
public void printDiscoveredParticipants() {
       ParticipantBuiltinTopicData participantData = new ParticipantBuiltinTopicData();;
       SampleInfo info = new SampleInfo();;
       
       try {
           while ( true ) {          
               participantsDR.take_next_sample(participantData, info);

               if ( info.instance_state == InstanceStateKind.ALIVE_INSTANCE_STATE ) {
                   System.out.println("Participant (New)" +
                           " messageNum: " + info.reception_sequence_number.low +
                           " name: \"" + participantData.participant_name.name + "\"" +
                           " created at: " + info.source_timestamp +
                           " detected at: " + info.reception_timestamp +
                           " source sn: " + info.publication_sequence_number.low +
                           " handle: " + info.instance_handle.toString() +
                           " full details: " + participantData.toString());
               } else {
                   String dissapearReason;
                   if ( info.instance_state == InstanceStateKind.NOT_ALIVE_DISPOSED_INSTANCE_STATE  ) {
                       dissapearReason = "deleted";
                   } else {
                       dissapearReason = "lost connection";
                   }     
                   if ( info.valid_data ) {
                       System.out.println("Participant (Dissapeared - "+ dissapearReason +"):" +
                               " messageNum: "  + info.reception_sequence_number.low +
                               " name: \"" + participantData.participant_name.name + "\"" +
                               " detected at: " + info.reception_timestamp +
                               " source sn: " + info.publication_sequence_number.low +
                               " handle: " + info.instance_handle.toString() +
                               " full details: " + participantData.toString());
                   }
                   else {
                       System.out.println("Participant (Dissapeared - "+ dissapearReason +"):" +
                               " messageNum: "  + info.reception_sequence_number.low +
                               " source sn: " + info.publication_sequence_number.low +
                               " handle: " + info.instance_handle.toString() +
                               " detected at: " + info.reception_timestamp );                     
                   }
               }
           }

       } 
       catch (RETCODE_NO_DATA noData) {  } 

       finally {  }
   }
\endcode
     *
     * Also, this is one of the three locations where the Participant GUID 
     * appears as such: the DDS_ParticipantBuiltinTopicData::key is the 
     * participant's GUID.
     * (compare http://community.rti.com/forum-topic/get-info-about-participants-losts)
     * 
     * (The two other locations are DDS_PublicationBuiltinTopicData::participant_key 
     * and DDS_SubscriptionBuiltinTopicData::participant_key).
     * 
     */
    virtual void on_valid_data_available ( DDS_ParticipantBuiltinTopicData& data,
                                           DDS_SampleInfo& info,
                                           DDSParticipantBuiltinTopicDataDataReader* reader )
    {
      
      if( true )
      { // Debug Output
        QString participantDetails;
        if(info.instance_state == DDS_ALIVE_INSTANCE_STATE )
        { participantDetails = "(Alive)"; }
        else if( info.instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE )
        { participantDetails = "(Dissapeared - Deleted)"; }
        else if( info.instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE )
        { participantDetails = "(Dissapeared - Lost Connection)"; }
        else
        { /* there are no other options in DDS_InstanceStateKind. */}
        
//         qDebug() << parent()->objectName() <<": (BuiltInTopic_Participant_Listener)"
//           << "Participant" << participantDetails <<"\n"
//           << "messageNum:" << info.reception_sequence_number.low << "\n"
//           << "name:" << data.participant_name.name << "\n"
//           << "created at:" << info.source_timestamp << "\n"
//           << "detected at:" << info.reception_timestamp << "\n"
//           << "source sequence number:" << info.publication_sequence_number.low << "\n"
//           << "handle:" << info.instance_handle;
           
//             << "full details:";
//         DDS_ParticipantBuiltinTopicDataTypeSupport_print_data(&data);
      }
      
      DDS_Long lease_duration_sec = -1;
      if( !data.lease_duration.is_infinite() )
      { lease_duration_sec = data.lease_duration.sec; }
      
      auto propertyHash = DDS::propertyToStringHash( data.property.value );
      auto propertyList = DDS::propertyToStringList( data.property.value );
      
      if( info.view_state == DDS_NEW_VIEW_STATE )
      {  
        qDebug() << parent()->objectName() << ": (BuiltInTopic_Participant_Listener) Participant detected:"
          << "key" << data.key
          << " with name "<< QString(data.participant_name.name)
          << ", type "<< propertyHash["type"]
          << ", and a lease duration of " << lease_duration_sec << " seconds.";
         
        NodeId id(data.key); //NOTE: conversion from a BuiltinTopicKey_t
//         qDebug() << parent()->objectName() << ": Participant  detected:"
//           << "id =" << id;
        
        Node::Type type = qStringToNodeTypeEnum( propertyHash["type"]);
        QString name(data.participant_name.name);
                
        // post an event to the queue of Node
        QMetaObject::invokeMethod(parent(),"addNewBlip",Qt::AutoConnection,
          Q_ARG(NodeId,      id),
          Q_ARG(Node::Type,  type ),
          Q_ARG(QString,     name ),
          Q_ARG(QStringList, propertyList )
         );
      }
      else
      { 
        qWarning() << parent()->objectName() << ": (BuiltInTopic_Participant_Listener) Participant recognized (view_state is not new):"
          << "key" << data.key
          << " with name "<< QString(data.participant_name.name)
          << ", type "<< propertyHash["type"]
          << ", and a lease duration of " << lease_duration_sec << " seconds.";
          
//         Q_ASSERT( parent()->blipExists(NodeId(data.key)) ); 
      }
    };
    
    virtual void on_sample_info_available ( DDS_SampleInfo& info, DDSParticipantBuiltinTopicDataDataReader* reader )
    {
//       qDebug() << parent()->qualifiedName() << "(BuiltInTopic_Participant_Listener)"
//         << ": reached 'on_sample_info_available.'";
      Base::on_sample_info_available(info,reader);
      
      if( info.instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE ) // deleted
      {
        NodeId id(info.instance_handle); //NOTE: conversion from a BuiltinTopicKey_t
        
        qDebug() << parent()->objectName() << ": A DDS Participant has been deleted:"<<id;
        
        Node* parentNode = dynamic_cast<Node*>( parent() );
        
        if( parentNode and !QMetaObject::invokeMethod( parentNode,"deleteBlip",Qt::AutoConnection,
                                        Q_ARG(NodeId,id)) )
        { qDebug() << parent()->objectName() << ": Could not invoke the deletion of the associated Blip."; }

      }
      else if (info.instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE ) // lost connections
      {
        NodeId id(info.instance_handle); //NOTE: conversion from a BuiltinTopicKey_t
        
        qDebug() << parent()->objectName() << ": A DDS Participant has lost its connection:"<< id;
        
        Node* parentNode = dynamic_cast<Node*>( parent() );
        
        if( parentNode and !QMetaObject::invokeMethod( parentNode,"deleteBlip",Qt::AutoConnection,
                                        Q_ARG(NodeId,id)) )
        { qDebug() << parent()->objectName() << ": Could not invoke the deletion of the associated Blip."; }

      }
    };
    
  };
  
  
  
  /** \brief Catch-all Listener at the Participant level.
   * \details From the RTI API documentation:
   * "The purpose of the DDSDomainParticipantListener is to be the listener of 
   * last resort that is notified of all status changes not captured by more 
   * specific listeners attached to the DDSDomainEntity objects. When a relevant
   * status change occurs, RTI Connext will first attempt to notify the listener
   * attached to the concerned DDSDomainEntity if one is installed. Otherwise, 
   * RTI Connext will notify the Listener attached to the DDSDomainParticipant."
   *
   * \note The simple presence of the callbacks doesn't mean that they indeed are
   * triggered. Check the DDS_StatusMask associated with each listener to ensure 
   * that all callbacks implemented are actually triggered, too.
   * This normally is handled by a set_listener(listener,mask) routine.
   */
  class Participant_Listener : public DDSDomainParticipantListener
  {
  public:
    /** \brief A status mask capturing all implemented callback routines. */
    static const DDS_StatusMask implementedCallbacks = 0
      | DDS_LIVELINESS_CHANGED_STATUS 
      | DDS_LIVELINESS_LOST_STATUS
      | DDS_RELIABLE_READER_ACTIVITY_CHANGED_STATUS
//       | DDS_RELIABLE_WRITER_CACHE_CHANGED_STATUS
      ;
    
    
  public:
    Participant_Listener(Node* const parent = nullptr)
      :m_parent_node(parent)
    {};
    
    
    virtual void on_liveliness_changed( DDSDataReader* reader,
      DDS_LivelinessChangedStatus const & status)
    {  
//       qDebug() << m_parent_node->qualifiedName() << "(Participant_Listener)"
//         << ": DataReader liveliness changed for topic" << reader->get_topicdescription()->get_name() << ".\n"
//         << "    alive count:" << status.alive_count << "(change:"<< status.alive_count_change << ")\n"
//         << "not alive count:" << status.not_alive_count << "(change:"<< status.not_alive_count_change << ")";
//         << "last publication handle :" << status.last_publication_handle << "\n";
      
      { /// remove the stale participant
        auto m_parent_node_private = m_parent_node->d_func();
        DDS_InstanceHandle_t staleParticipantHandle = status.last_publication_handle;
        
        if( m_parent_node_private->detectedParticipants.contains( staleParticipantHandle ) )
        {
          auto staleParticipantId = m_parent_node_private->detectedParticipants.value( staleParticipantHandle );
          
          qDebug() << m_parent_node->qualifiedName() 
            << ": Removing a stale DDS participant. (" << m_parent_node->blip(staleParticipantId).name() <<")";
          
          m_parent_node_private->deleteBlip( staleParticipantId );
          m_parent_node_private->detectedParticipants.remove( staleParticipantHandle );
        }
      }
    };
    
    virtual void on_liveliness_lost( DDSDataWriter *writer, 
      DDS_LivelinessLostStatus const & status)
    {      
//       qDebug() << m_parent_node->qualifiedName() 
//         << ": DataWriter liveliness lost for topic" <<writer->get_topic()->get_name() << ".\n"
//         << "total count:" << status.total_count << "(change:"<< status.total_count_change <<")";
    };
    
    virtual void on_reliable_reader_activity_changed( DDSDataWriter* writer,
      const DDS_ReliableReaderActivityChangedStatus &   status )   
    {
//       qDebug() << m_parent_node->qualifiedName() 
//         << ": ReliableReader activity changed for topic" << writer->get_topic()->get_name() << ".\n"
//         << "  active count:" << status.active_count   << "(change:" << status.active_count_change << ")\n"
//         << "inactive count:" << status.inactive_count << "(change:" << status.inactive_count_change << ")";
    };
    
    virtual void on_reliable_writer_cache_changed( DDSDataWriter* writer,
      const DDS_ReliableWriterCacheChangedStatus& status )
    {
//       qDebug() << m_parent_node->qualifiedName() 
//         << ":ReliableWriter cache changed for topic" << writer->get_topic()->get_name() << ".\n"
//         << "empty writer cache:" <<  status.empty_reliable_writer_cache.total_count
//         << "(change:" << status.empty_reliable_writer_cache.total_count_change << ")\n"        
//         << " full writer cache:" << status.full_reliable_writer_cache.total_count
//         << "(change:" << status.full_reliable_writer_cache.total_count_change << ")\n"
//         << " low watermark cache:" << status.low_watermark_reliable_writer_cache.total_count 
//         << "(change:" << status.low_watermark_reliable_writer_cache.total_count_change << ")\n"        
//         << "high watermark cache:" << status.high_watermark_reliable_writer_cache.total_count
//         << "(change:" << status.high_watermark_reliable_writer_cache.total_count_change << ")\n"        
//         << "unacknowledged sample count:" << status.unacknowledged_sample_count
//         << "(peak:" << status.unacknowledged_sample_count << ")";
    };

    
  private:
    Node* const m_parent_node;
  
  };
                 
  //@}
  
  };

Q_DECLARE_METATYPE(NodePrivate::RfZone);

#endif // NODEPRIVATE_H
