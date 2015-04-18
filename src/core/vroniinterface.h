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

#ifndef VRONIINTERFACE_H
#define VRONIINTERFACE_H


#include <string>
#include <functional>
#include <unordered_map>

#include "config.h"

#include "Geometry/geometry.h"
#include "Graph/simple2dgraph.h"
#include "Graph/routegraph.h"
#include "Graph/vronigraph.h"
#include "Environment/obstacle.h"

namespace VRONI { struct edge; } // forward declaration

/** \brief This class operates as an interface to the VRONI Library.
 * 
 * All calls related to VRONI originate from within this class.
 * The original VRONI code is contained in a namespace of the same name, VRONI::
 */
class VroniInterface
{
public:
  /** \brief Default Constructor 
   * The constructor also initializes the underlying VRONI code via a call to 
   * \c VRONI::API_InitializeProgram()
   */
  VroniInterface();
  
  /** \brief Destructor 
   * The desctructor calls \c VRONI::API_TerminateProgram() to clean up. This 
   * will also free up all memory allocated by VRONI.
   */
  virtual ~VroniInterface();
  
  /** \brief Contains the references needed to scale 2D objects.
   *
   * When the boundary is set, the lexilexicographically smallest vertex 
   * \f$(x_{min},y_{min})\f$ is mapped to the \f$(0,0)\f$ origin of the 
   * numerical coordinate system, thelexilexicographically largest vertex 
   * \f$(x_{max},y_{max})\f$ is mapped to \f$(1,1)\f$. 
   * 
   * The scaling factors hence can be computed as 
   * \f$ x_{scale}=\frac{1}{x_{max}-x_{min}}\f$ and \f$ y_{scale}=\frac{1}{y_{max}-y_{min}}\f$,
   * respectively.
   * 
   * \internal Remeber: Order matters! The constructor does not initialize
   *  member variables in the order given after the ":", but in the order they
   *  were introduced in the declaration!
   */
  struct ScalingBox {
    double x_min; ///< \brief The maximum x-value of the boundary bounding box.
    double y_min; ///< \brief The maximum y-value of the boundary bounding box.
    double x_max; ///< \brief The minimum x-value of the boundary bounding box.
    double y_max; ///< \brief The minimum y-value of the boundary bounding box.
    double x_scale; ///< \brief The scaling factor for the x-axis.
    double y_scale; ///< \brief The scaling factor for the y-axis.
    bool scaleIsSet; ///< \brief Internal flag to keep track of whether scale / bounding box has been set.
    ScalingBox()
      :x_min(0.0),y_min(0.0),x_max(0.0),y_max(0.0)
      ,x_scale(1.0/(x_max-x_min)),y_scale(1.0/(y_max-y_min))
      ,scaleIsSet(false) {}; ///< \brief Default Constructor (invalid result)
    ScalingBox(const double& x_min, const double& y_min,
               const double& x_max, const double& y_max)
      :x_min(x_min),y_min(y_min),x_max(x_max),y_max(x_max)
      ,x_scale(1.0/(x_max-x_min)),y_scale(1.0/(y_max-y_min))
      ,scaleIsSet(true) {}; ///< \brief Construct and initialize to values.
//     ScalingBox(const VisiLibity::Scaling_Box& vlBbox, const double& padding=0.0)
//       :x_max(vlBbox.x_max+padding),x_min(vlBbox.x_min-padding)
//       ,y_max(vlBbox.y_max+padding),y_min(vlBbox.y_min-padding)
//       ,x_scale(1.0/(x_max-x_min)),y_scale(1.0/(y_max-y_min))
//       ,scaleIsSet(true)  {}; ///< \brief Construct from a VisiLibity::Scaling_Box.
    ~ScalingBox() {};///< \brief Empty destructor.
  };
  
  /** \brief A struct holding all necessary parameters for a VD/DT computation.
   */
  struct VDDTParameter {
    VDDTParameter()
      :save_input(false),new_data(true),timing(false),bound(1),sample(0)
      ,approx(0),check_for_duplicates(false),recover_arcs(true),auto_apx(true)
      ,apx_absolute(0.0),apx_relative(0.0),points_only(false)
      ,save_point_output(false),clean_data(false)
    {};
    bool save_input = false;  ///< \brief TRUE if the input site data should be saved to a file (default: FALSE)
    bool new_data = true;  ///< \brief TRUE if no previous VD/DT data for any parts of the site excist (default: TRUE)
    bool timing = false;  ///< \brief TRUE if the VD/DT computation should be timed (default: FALSE)
    int bound = 1;    ///< \brief bounding box scaling factor (default: 1)
    int sample = 0;   ///< \brief sampling factor for segments/arcs (default: 0)
    int approx = 0;   ///< \brief approximation factor for circular arcs (default: 0)
    std::string site_file; ///< \brief file name to store the input site in if "save_input"==TRUE (default: "")
    bool check_for_duplicates = false; ///< \brief TRUE if VRONI should check for duplicate data in the site. (default: FALSE)
    bool recover_arcs = true;  ///< \brief TRUE if an approximate circular arc should be computed (where appropriate) and continued to be used thereafter (default: TRUE)
    bool auto_apx = true;  ///< \brief TRUE if Held's heuristic should be used to compute "apx_absolute" and "apx_relative"
    double apx_absolute = 0.0; ///< \brief approximation threshold for circular arcs (default: 0.0)
    double apx_relative = 0.0; ///< \brief approximation threshold for circular arcs (default: 0.0)
    bool points_only = false;   ///< \brief compute the VD/DT only for point sites
    bool save_point_output = false; ///< \brief TRUE if the VD/DT results of a points_only compuation should be saved to a file
    std::string points_VDDT_file; ///< \brief file name to store the VD/DT output in if "points_only" and "save_point_output" both are TRUE
    bool clean_data = false; ///< \brief TRUE if the internal site data should be cleaned before a VD/DT computation (default: FALSE)
  };
  
  /** \brief A struct holding all necessary parameter for a WMAT computation.
   */
  struct WMATParameter {
    WMATParameter()
      :auto_WMAT(true),WMAT_angle(0.0),WMAT_distance(0.0),left(true)
      ,right(false),timing(false)
    {};
    bool auto_WMAT = true;     ///< \brief TRUE if "WMAT_angle" and "WMAT_distance" should be computed by Held's heuristic (default: TRUE)
    double WMAT_angle = 0.0;    ///< \brief Angle threshold in radians, valid interval [0, pi]. (default: 0.0)
    double WMAT_distance = 0.0;   ///< \brief Distance threshold in site-identical units. (default: 0.0)
    bool left = true;      ///< \brief TRUE if the WMAT should be computed on the left side of segments, i.e. in between the obstacles (default: TRUE)
    bool right = false;     ///< \brief TRUE if the WMAT should be computed on the right side of segments, i.e. in the interior of the obstacles (default: FALSE)
    bool timing = false;    ///< \brief TRUE if the computation of the WMAT should be timed. (default: FALSE)
  };

  /** \brief Use \c VRONI::API_ResetAll() to reset the VRONI internal site data.
   * 
   * Call this routine whenever new data is to be input and VRONI's data 
   * structures are  to be reset.
   * Calling this function will set VroniInterface::m_vd_is_current to \c false 
   * and VroniInterface::m_isPointSite to \c true.
   * 
   * \note This function won't free allocated memory.
   */
  void resetAll(void);
  
  /** \brief (Re-)Set the VRONI internal site data to the given shape(s). 
   * The function clears any prior site data and creates a new site from 
   * the given data, expressend in a complex polygon (i.e., it can contain 
   * holes)
   */
  void setVroniSiteData(const Geometry::Polygon_2D& complexPolygon);
  
  /** \brief Set the VRONI internal site data to the given shape(s)/
   * 
   * Set the VRONI internal site data and get a map associating the introduced
   * VRONI segments to the related obstacles.
   * An example: the first segement of obstacle \c i would be 
   * <code>VRONI::segs* s = VRONI::segs+obstacleSegmentMap(i);</code>
   * 
   * \note I appears that the VRONI site data (particularly the PNT data) is 
   * cleaned up during the calls to compute the VD. As such, don't use the
   * PNT data before that happens, i.e. only when m_vdIsCurrent == true.
   * 
   * \pre boundary is larger than the convex hull of all obstacles.
   * \param[in] boundary The outer boundary of the site. 
   * \param[in] obstacleData The obstacles that shall be added as "holes".
   * \return A map holding the index of the first VRONI segment associated with an 
   * obstacle.
   */
  void setVroniSiteData(const Geometry::Polygon_2D& boundary, const std::unordered_map< ElementId, Obstacle>& obstacleData);
  
  void setVroniSiteData(const Geometry::MultiPoint_2D& pointSet);
  
  /** \brief Use \c VRONI::API_ArrayInput() to add a shape to VRONI's internal site data.
   * 
   * \note It is \e not necessary to scale input data into VRONI's internal 
   *  coordinate space of \f$ \{[0,1]\times[0,1]\} \f$, VRONI will take care of
   *  that internally.
   * 
   * 
   * \param[in] ring A const ref to the data that will be added to the VRONI 
   *  internal representation of the site. (Must fullfill the Boost Geometry 
   *  Ring concept.)
   * \param[i   * 
   * \sa VRONI::Setn] data Additional data stored with the created VRONI segments.
   */
  void addElementToSite(const Geometry::Polygon_2D::ring_type& ring, VRONI::eas_type data = VRONI::eas_NIL);
  
  /** \overlaod */
  void addElementToSite(const Geometry::Point_2D & point);
  
  /** \overlaod */
  void addElementToSite(const Geometry::MultiPoint_2D & mulitPoint);
  
  
  using EdgeDataMap=boost::iterator_property_map<
    VRONI::edge*,
    VroniGraph::Edge_VroniIndex_Map,
    VRONI::edge,
    VRONI::edge&    
  >;

  using NodeDataMap = boost::iterator_property_map<
    VRONI::node*,
    VroniGraph::Vertex_VroniIndex_Map,
    VRONI::node,
    VRONI::node&
  >;

#ifdef WITH_GRAPHICS
  
  /** \brief Get graphics representing the Point, Segement, and Arc site data of VRONI. */
  QGraphicsItemGroup* getSiteGraphics();
  
  /** \brief Get graphics representing all computed VD nodes. */
  QGraphicsItemGroup* getNodeGraphics();
  
  /** \brief Get graphics representing all computed VD edges. */
  QGraphicsItemGroup* getEdgeGraphics();
  
#endif // WITH_GRAPHICS  
  
  Simple2dGraph::Graph createSiteShapeAssociationGraph();

  /** \brief Compute a simple graph showing the relation between the VD edges and
   * the input sites. 
   *
   * The idea is that each site element (PNT, SEG, or ARC) as well as each VD
   * edge is a vertex in this graph. Edge-vertices are connected to 
   * site-vertices if VRONI has the site as the left or right defining element 
   * of that edge.
   * 
   * All vertices have a 2D position, where SEG, ARC, and edge elements are 
   * represented at the median of the two defining node/PNT locations.
   *
   */
  Simple2dGraph::Graph createEdgeSiteAssociationGraph();
  
  
  Simple2dGraph::Graph createEdgeShapeAssociationGraph(QHash<int,int> shapeToEdgeMap);
  
  /** \deprecated{Use a combination of determineVoronoiCells() and 
   * createEdgeShapeAssociationGraph() } */
  Simple2dGraph::Graph createEdgeShapeAssociationGraph();
  
  /** \brief Use \c VRONI::API_computeVD() to compute the VD/DT of the current site.
   * 
   * This function causes VRONI to compute the Voronoi diagram and the 
   * corresponding Delaunay triangulation based on the currently held internal 
   * site representation.
   * 
   * Necessary parameters are handed in via a parameter struct of the type
   * VroniInterface::VDDTParameter.
   * \param params
   */
  void computeVD(const VroniInterface::VDDTParameter& params = VroniInterface::VDDTParameter()); 
  
  /** \brief Use VRONI's API_ComputeWMAT() to compute the (weighted) medial axes data.
   * 
   * This function computes the (weighted) medial axes (WMAT) for the data 
   * currently held in VRONI's internal site representation.
   * The medial axes are the paths "of maximum clearance" through the site.
   * 
   * This function requires that a call to \c VRONI::API_ComputeCD() has happened 
   * beforehand. If necessary, the funtion will make this call via an internal 
   * call to VroniInterface::computeVD(), using the default VD/DT parameters 
   * generated by the constructor of VroniInterface::VDDTParameter.
   * 
   * \sa VroniInterface::computeVD()
   *  
   * \param WMAT_distance  Distance threshold in site-identical units.
   *   See the VRONI documentation "WMAT.pdf" for an explanation of 
   *   this parameter.
   * \param WMAT_angle Angle threshold in radians, valid interval [0, pi].
   *   (optional) See the VRONI documentation "WMAT.pdf" for an explanation of 
   *   this parameter.
   * \param auto_WMAT Automatic paramater computation.
   *   (optional) TRUE if Held's heuristic should be used to compute WMAT 
   *   parameters. In this case the user supplied parameters "WMAT_angle" and 
   *   "WMAT_dist" are ignored.
   * \param left
   *   (optional) TRUE if the WMAT should be computed on the left side of
   *   input segments. If neither "left" nor "right" (or both) are TRUE, the 
   *   WMAT is computed on both sides, left and right.
   * \param right
   *   (optional) TRUE if the WMAT should be computed on the right side of
   *   input segments.  If neither "right" nor "left" (or both) are TRUE, the 
   *   WMAT is computed on both sides, left and right.
   * \param timing 
   *   (optional) TRUE if the compuation of the WMAT should be timed.
   */
  void computeWMAT(double WMAT_distance, double WMAT_angle = 0,  bool auto_WMAT=false, 
       bool left=true, bool right=false, bool timing=false);
  /** \overload
   * 
   * \param params A struct holding all necessary parameters
   * 
   * @see VroniInterface::WMATParameter
   * @see VroniInterface::computeVD()
   */
  void computeWMAT(const VroniInterface::WMATParameter& params = VroniInterface::WMATParameter());
 
//   /** \brief Extract data from the Voronoi computation and generate a graph from it.
//    * 
//    * \return 
//    * 
//    */
//   VroniGraph::Graph extractGraph(std::function<bool (VRONI::edge*)> inclusion_check ) const;
  
  /** \brief Extract the Voronoi diagram from the current data. 
   * \param[in] inclusion_check The lambda used to decide wether or not to inlcude 
   * a specific VRONI edge into the returned graph. 
   * \return A boost graph representation of the (subset of the) VRONI data.
   * \sa extractCompleteVdGraph
   * \sa extractWmatGraph
   * \sa extractNonWmatGraph
   */
  VroniGraph::Graph extractVdGraph(std::function<bool (VRONI::edge*)> inclusion_check) const;
  
    
  /** \brief Convenience function to get the complete VD graph. */
  VroniGraph::Graph extractVdGraph(void) const;

  /** \brief Convenience function to get the the WMAT part of the VD graph. */
  VroniGraph::Graph extractWmatGraph(void) const;
  
  /** \brief Convenience function to get the non-WMAT part of the VD graph. */
  VroniGraph::Graph extractNonWmatGraph(void) const;
  
  /** \brief Get a planar embedding of the VdGraph.
   * 
   * \tparam Graph A type to hold an extracted Vd graph.
   * \tparam Embedding A type that models the Planar Embedding concept. See
   * http://www.boost.org/doc/libs/1_52_0/libs/graph/doc/PlanarEmbedding.html
   * \param[in] graph The previously extracted VD graph to create the planar 
   * embedding for.
   * \param[in] inclusion_check The lambda used to decide wether or not to inlcude 
   * a specific VRONI edge into the returned graph. I should (well, must) be
   * the same as the one used to extract graph.
   * \param[out] embedding The planar embedding of the graph.
   * \return TRUE if the creation of the embedding was successful; FALSE 
   * otherwise.
   */
  template< typename Graph
           ,typename Embedding
          >
  bool extractPlanarEmbedding( Graph& graph
                              ,Embedding& embedding
                              ,std::function<bool (VRONI::edge*)> inclusion_check
                             ) const
  {  
    /** \internal The VRONI data structures already contain all the information
     * related to a planar embedding. The information is mainly contained inside
     * the VRONI::edge structure, by means fo the s_ccw, s_cw, e_ccw, and e_cw
     * integers.
     */
    
    using namespace boost;
    
    auto vDataMap   = get(vertex_vroniNodeData, graph); ///<  Vertex_VroniNodeData_Map 
    auto vVroniMap  = get(vertex_vroniIndex,graph); ///< Vertex_Index_Map
    auto eVroniMap  = get(edge_vroniIndex, graph); ///<  Edge_VroniIndex_Map 
 
    bool success = true; // keeps a record of the success of the algorithm.
    
    // invert the eVroniMap
    QHash<int,typename Graph::edge_descriptor>   index_to_edge_map;  
    typename graph_traits<Graph>::edge_iterator ei, ei_end;
    for(tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
    { index_to_edge_map[eVroniMap[*ei]] = *ei; }

//     // Define the storage type for the planar embedding
//     using embedding_storage_t = std::vector< std::vector< typename graph_traits<Graph>::edge_descriptor > > ;
//     // Define the property map interface to the storage
//     using embedding_pMap_t =  iterator_property_map
//       < typename embedding_storage_t::iterator, /// the underlying data structure holding the property 
//         typename property_map<Graph, vertex_index_t>::type /// the offset map used to translate vertex_descriptor to property
//       >;
// 
//     // Create the planar embedding
//     embedding_storage_t embedding_storage(num_vertices(graph));
//     embedding_pMap_t embedding(embedding_storage.begin(), vIndexMap);  

    // iterate over all vertices in the graph
    typename graph_traits<Graph>::vertex_iterator vi, vi_end;
    for(tie(vi, vi_end)=vertices(graph); vi != vi_end; ++vi)
    {
      // get the VRONI node data for this vertex
      VRONI::node vr_node = vDataMap[*vi];
      
      // pick an incident edge to start from
      const int i_start_edge = vr_node.edge;
      int i_vr_edge = i_start_edge;    
      
      embedding[*vi].clear();
      
      /** \internal One cannot simply use any of the "next edge in ..." integers
        * of the edge structure to create the embedding. The boost embedding
        * contains the edges around a certain vertex, but the VRONI data is 
        * stored in the edges structure which differentiates between the start 
        * and the end node. That means that in order to find the next edge one
        * first needs to determine wether the vertex curretnly being walked 
        * around is the start or end node of the currently processed edge.
        */
      do
      {
        // get the VRONI edge to check for inclusion
        VRONI::edge* vr_edge = VRONI::edges + i_vr_edge;
        
        // only include a certain subset into the embedding
        if(inclusion_check(vr_edge)==true)
        { 
          // get the edge descriptor for the edge and push it into the embedding.
          typename graph_traits<Graph>::edge_descriptor edge = index_to_edge_map[i_vr_edge];
          embedding[*vi].push_back(edge);
        }
        
        // determine wether to get the next edge around the start or the end node
        // of the current edge        
        if( vr_edge->n1 == vVroniMap[*vi] ) 
        { // pivot ccw around the start node
          i_vr_edge = vr_edge->s_ccw;
        }
        else if( vr_edge->n2 == vVroniMap[*vi] )
        { // pivot ccw around the end node
          i_vr_edge = vr_edge->e_ccw;
        }
        else
        {
          qDebug() << "The VRONI interface cannot determine the planar embedding."
            << "(Error at node" << vVroniMap[*vi] << ")";
          success = false;
          break;
        }
      }
      while( i_vr_edge != i_start_edge );
      
    }
    
    return success;
    
  };


  /** \brief Extract the Delaunay triangulation for a point-only site. */
  VroniGraph::Graph extractDtGraph(void) const;  
  
  QHash<int,int> determineVoronoiCells();
  
//   QHash<int,Geometry::Polygon_2D> extractVoronoiCellGeometries() const;
   
  void printVroniSiteData();
  
  bool isVdCurrent() const;
   
private:
  Q_DISABLE_COPY(VroniInterface)
    
  /** \brief Get the VRONI bounding box. 
   * Internally VRONI maps box.min_corner() to \f$(0,0)\f$ and 
   * box.max_corner() to \f$(1,1)\f$ for improved numerical precision. 
   */
  Geometry::Box_2D boundingBox() const;
  
  /** \brief Transform a coordinates from the unit square to the "original" coordinates.
   * \param[in] point The point in unit square coordinates, i.e. 
   *  \f$ \in \{[0,1]\times[0,1]\} \f$
   * \return A point in the orignal coordinates, i.e. \f$ \in \f$ boundingBox()
   */
  Geometry::Point_2D unScale(const Geometry::Point_2D& point) const;
  
//   /** \brief Transform a coordinates from the unit square to the "original" coordinates.
//    * \overload
//    * \param[in,out] The point to be scaled from \f$ \in \{[0,1]\times[0,1]\} \f$
//    *  to boundingBox() coordinates.
//    */
//   void unScale(Geometry::Point_2D& point) const;
  
  
  /** \brief Keeps track if the Voronoi diagram is current wrt. the internal site data */
  bool m_vdIsCurrent = false;
  
  bool m_isPointSite = false;
  
  ScalingBox m_scalingBox;
  
  void get_pnt_coords(Geometry::Point_2D & point, int const & pntIndex) const;
  void get_node_coords(Geometry::Point_2D & point, int const & nodeIndex) const;
  
  QHash<int,Geometry::Point_2D> shapeCentroidMap;
  
  QHash<int,int> getSegToShapeMap() const;
  QHash<int,int> getPntToShapeMap() const;
  
  std::function<bool (VRONI::edge*)> always_true = [](VRONI::edge* e) {return true;};
  std::function<bool (VRONI::edge*)> in_w_mat = [](VRONI::edge* e) {return e->w_mat.in_w_mat;};
  std::function<bool (VRONI::edge*)> in_vddt = [](VRONI::edge* e) {return !e->w_mat.in_w_mat;};
   
};








#endif // VRONIINTERFACE_H
