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

#include "vroniinterface.h"

/* VRONI header files
 *   These header files are part of the VRONI source code
 *   NOTE: The order of inlcusion does matter!
 */
#include <vroni.h>

#include <boost/geometry/algorithms/envelope.hpp> 
#include <boost/geometry/algorithms/reverse.hpp> 
#include <QtCore/QHash>
#include <QtCore/QDebug>

#include <unordered_map>

#include "Graphics/arrowitem.h"

VroniInterface::VroniInterface()
{
  // from VRONI docu:
  // call this routine once prior calling any other routine of VRONI
  VRONI::API_InitializeProgram();
  
  VRONI::quiet = true; // suppress printfs
}

VroniInterface::~VroniInterface()
{
  // from VRONI docu:
  // call this routine to release all memory allocated by VRONI
  VRONI::API_TerminateProgram();
}

void VroniInterface::resetAll(void )
{
  // call VRONI's API funtion
  VRONI::API_ResetAll();
  
  // mark the (now erased) VD/DT data as not current
  m_vdIsCurrent = false;
  m_isPointSite = true;
}


void VroniInterface::setVroniSiteData(const Geometry::Polygon_2D& complexPolygon)
{
  resetAll(); // reset VRONI's data...
  m_isPointSite = false;
  
  namespace bg = boost::geometry;
  
  // get the containing box of the polygon
  bg::model::box<Geometry::Point_2D> box;
  bg::envelope(complexPolygon,box);
  
  // set the scaling box from that
  auto x_min = box.min_corner().get<0>();
  auto y_min = box.min_corner().get<1>();
  auto x_max = box.max_corner().get<0>();
  auto y_max = box.max_corner().get<1>();
 
  m_scalingBox = ScalingBox(x_min,y_min,x_max,y_max); //NOTE: Warning, this could include an implicit conversion to double!
  
 
  // add the outer polygon
  addElementToSite( complexPolygon.outer());
  
  // add the inner holes, i.e. the obstacles
  auto obstacleList =  complexPolygon.inners();
  for(Geometry::Polygon_2D::ring_type const & ring : obstacleList)
  {
    addElementToSite(ring);
  }
  
  
  boundingBox();
}

void VroniInterface::setVroniSiteData(const Geometry::Polygon_2D& boundary, const std::unordered_map<ElementId, Obstacle >& obstacleData)
{
  resetAll(); // reset VRONI's data...
  m_isPointSite = false;
  
//   qDebug() << "ENTERING setVroniSiteData";
//   printVroniSiteData();
  
  namespace bg = boost::geometry;
  
  // get the containing box of the polygon
  bg::model::box<Geometry::Point_2D> box;
  bg::envelope(boundary,box);
  
  // set the scaling box from that
  auto x_min = box.min_corner().get<0>();
  auto y_min = box.min_corner().get<1>();
  auto x_max = box.max_corner().get<0>();
  auto y_max = box.max_corner().get<1>();
 
  m_scalingBox = ScalingBox(x_min,y_min,x_max,y_max); //NOTE: Warning, this could include an implicit conversion to double!
  
 
  // add the outer boundary to the site and add the shape identifier "0"
  int index = 0;
  
  Geometry::Point_2D centroid;
  boost::geometry::centroid(boundary.outer(),centroid);
  shapeCentroidMap.clear();
  shapeCentroidMap[index] = centroid;
  
  addElementToSite( boundary.outer(), index++);
  
  // add the obstacles to the site_file
  for( auto& pair : obstacleData  ) 
  {
    Obstacle const& obstacle = pair.second;
    
    //NOTE: we need to make a copy of the outer obstacle shape so that  we can 
    // use bg::reverse. This is important so that the boundary and the obstacle 
    // shapes have different orientations so that VRONI can correctly determined
    // "holes" and the "inside" of the arena.
    Geometry::Polygon_2D::ring_type tempRing;
    bg::assign(tempRing,obstacle.shape().outer());
    bg::reverse(tempRing);
    
    Geometry::Point_2D centroid;
    boost::geometry::centroid(tempRing,centroid);
    shapeCentroidMap[index] = centroid;

    addElementToSite( tempRing, index++ );
  }
  
  
  // update the bounding box
  boundingBox();
  
}

void VroniInterface::setVroniSiteData(const Geometry::MultiPoint_2D& mulitPoint)
{
  resetAll(); // reste VRONI's internal site data
  m_isPointSite = true;
  
  namespace bg = boost::geometry;
  
  // get the containing box of the point set
  boost::geometry::model::box<Geometry::Point_2D> box;
  boost::geometry::envelope(mulitPoint,box);

  // set the scaling box from that
  auto x_min = box.min_corner().get<0>();
  auto y_min = box.min_corner().get<1>();
  auto x_max = box.max_corner().get<0>();
  auto y_max = box.max_corner().get<1>();
 
  m_scalingBox = ScalingBox(x_min,y_min,x_max,y_max); //NOTE: Warning, this could include an implicit conversion to double!
  
  // add the points to the internal VRONI site data set
  addElementToSite(mulitPoint);

  boundingBox();
  
}


void VroniInterface::addElementToSite(const Geometry::Polygon_2D::ring_type& ring, VRONI::eas_type data)
{
  using namespace VRONI;
  
  m_isPointSite = false; //NOTE: this function adds non-points to the VRONI site
  
  // get the number of edges of the geometry
  std::size_t num_edges = boost::geometry::num_points(ring) - 1; // this assumes that the ring is closed!
  
  // preallocate a VRONI in_segs array to hold the edges' data
  in_segs* edges = new in_segs[num_edges];
  
  // populate the edges array
  int i=0;
  for( auto it_point=ring.begin() ; it_point !=--(ring.end()); /* no action */)
  {
    edges[i].x1 = boost::geometry::get<0>(*it_point); //NOTE: Warning: this could cause an implicit conversion
    edges[i].y1 = boost::geometry::get<1>(*it_point); //NOTE: Warning: this could cause an implicit conversion
    ++it_point;
    edges[i].x2 = boost::geometry::get<0>(*it_point); //NOTE: Warning: this could cause an implicit conversion
    edges[i].y2 = boost::geometry::get<1>(*it_point); //NOTE: Warning: this could cause an implicit conversion
    
    edges[i].ext_appl = data;
    ++i;
  }

  // store whether or not VRONI determined the API input to be new or not
  boolean new_input = false; 

  // Call the VRONI API to add the edge segments representing the QPolygon
  API_ArrayInput(
    0,nullptr,   // not adding any points
    num_edges, edges,   // adding num_edges segments, stored in/at edges
    0,nullptr,     // not adding any arcs
    &new_input    // check if VRONI determines this is a new input
  );
  
  if( new_input )
  {
    // reset the Voronoi diagram tracker
    m_vdIsCurrent = false;
  }
  
  // free the allocated memory temporarily holding the edges.
  // VRONIs API_ArrayInput performs an internal copy-by-value, hence this is 
  // safe
  delete[] edges;
}

void VroniInterface::addElementToSite(const Geometry::Point_2D& point)
{
  using namespace VRONI;
  
  // get/set the number of points to add
  int num_points = 1;

  // preallocate a VRONI in_pnts array to hold the points' data
  in_pnts * points = new in_pnts[num_points];
  
  // populate the points array
  int i = 0;
  {
    points[i].x1 =  boost::geometry::get<0>(point);
    points[i].y1 =  boost::geometry::get<1>(point);
  }
  
  
  // store whether or not VRONI determined the API input to be new or not
  boolean new_input = false; 
  
  // Call the VRONI API to add the edge segments representing the QPolygon
  API_ArrayInput(
    num_points, points,   // adding num_points elements, stored in/at points
    0,nullptr,   // not adding any edges
    0,nullptr,     // not adding any arcs
    &new_input    // check if VRONI determines this is a new input
  );
  
  if( new_input )
  {
    // reset the Voronoi diagram tracker
    m_vdIsCurrent = false;
  }
  
  // free the allocated memory temporarily holding the edges.
  // VRONIs API_ArrayInput performs an internal copy-by-value, hence this is 
  // safe
  delete[] points;
  
}

void VroniInterface::addElementToSite(const Geometry::MultiPoint_2D& mulitPoint)
{
  using namespace VRONI;
  
  // get/set the number of points to add
  int num_points = boost::geometry::num_geometries(mulitPoint);

  // preallocate a VRONI in_pnts array to hold the points' data
  in_pnts * points = new in_pnts[num_points];
  
  // populate the points array
  int i = 0;
  for( auto & point : mulitPoint )
  {
    points[i].x1 =  boost::geometry::get<0>(point);
    points[i].y1 =  boost::geometry::get<1>(point);
    ++i;
  }
  
  
  // store whether or not VRONI determined the API input to be new or not
  boolean new_input = false; 
  
  // Call the VRONI API to add the edge segments representing the QPolygon
  API_ArrayInput(
    num_points, points,   // adding num_points elements, stored in/at points
    0,nullptr,   // not adding any edges
    0,nullptr,     // not adding any arcs
    &new_input    // check if VRONI determines this is a new input
  );
  
  if( new_input )
  {
    // reset the Voronoi diagram tracker
    m_vdIsCurrent = false;
  }
  
  // free the allocated memory temporarily holding the edges.
  // VRONIs API_ArrayInput performs an internal copy-by-value, hence this is 
  // safe
  delete[] points;
}

#ifdef WITH_GRAPHICS

QGraphicsItemGroup* VroniInterface::getSiteGraphics()
{
  auto group = new QGraphicsItemGroup;
  
  QColor pntColor = Qt::red;    /// PNT site color
  QColor segColor = Qt::blue;   /// SEG site color
  QColor arcColor = Qt::green;  /// ARC site color
  
//   using LineGraphics = QGraphicsLineItem;
  using LineGraphics = Graphics_2D::Arrow;
  QPen segPen = QPen(segColor);
    
  using ArcGraphics = QGraphicsEllipseItem;
  QPen arcPen = QPen(arcColor);
  const bool approxArcsByLines = true;
  const bool drawArcCenter = true;
    
  using VertexGraphics = QGraphicsEllipseItem;
  QPen pntPen = QPen(pntColor);
  constexpr qreal radius = 4.0;

  // add segments
  for( int i = 0; i<VRONI::num_segs; ++i)
  {
    VRONI::seg* vr_seg  = VRONI::segs+i;
    
    Geometry::Point_2D tail;
    get_pnt_coords(tail,vr_seg->i1);
    
    Geometry::Point_2D head;
    get_pnt_coords(head,vr_seg->i2);
   
    auto line = new LineGraphics(tail.x(), tail.y(),
                                 head.x(), head.y());  
    
    line->setPen(segPen);
    group->addToGroup(line);
  }
  
  // add arcs
  for(int i = 0; i<VRONI::num_arcs; ++i)
  {
    VRONI::arc* vr_arc = VRONI::arcs+i; 
    
    if( approxArcsByLines )
    {
      Geometry::Point_2D tail;
      get_pnt_coords(tail,vr_arc->i1);
      
      Geometry::Point_2D head;
      get_pnt_coords(head,vr_arc->i2);
    
      
      if( boost::geometry::distance(tail,head) > ZERO )
      {
        auto line = new LineGraphics(tail.x(), tail.y(),
                                  head.x(), head.y());  
        group->addToGroup(line);     
      }
    }
    
    if( drawArcCenter )
    {
      Geometry::Point_2D center(vr_arc->c.x, vr_arc->c.y);
      if(VRONI::scale_data)
      { center = unScale(center); }
        
      auto point = new VertexGraphics(center.x()-radius/2, center.y()-radius/2,
                                      radius,radius);
    }
    
  }
     
  // add points  
  for( int i = 0; i<VRONI::num_pnts; ++i )
  {
    Geometry::Point_2D point;
    get_pnt_coords(point, i);
    
    auto vertex = new VertexGraphics(point.x()-radius/2, point.y()-radius/2,
                                      radius,radius);
    vertex->setPen(pntPen);
    group->addToGroup(vertex);
  }
      
  printVroniSiteData();
      
  return group;
}

QGraphicsItemGroup* VroniInterface::getNodeGraphics()
{
  auto group = new QGraphicsItemGroup;
  
  QColor nodeColor = Qt::darkRed;    
  
  int num_nodes = VRONI::GetNumberOfNodes();
  
  
  using VertexGraphics = QGraphicsEllipseItem;
  QPen nodePen = QPen(nodeColor);
  constexpr qreal radius = 4.0;
  
  // add nodes
  for( int i = 0; i<num_nodes; ++i )
  {
    Geometry::Point_2D node;
    get_node_coords(node,i);
   
    auto vertex = new VertexGraphics(node.x()-radius/2, node.y()-radius/2,
                                      radius,radius);
    vertex->setPen(nodePen);
    group->addToGroup(vertex);
   
  }
  
  return group;
}

QGraphicsItemGroup* VroniInterface::getEdgeGraphics()
{
  auto group = new QGraphicsItemGroup;
  
  QColor edgeColor = Qt::darkGreen;
  
  int num_edges = VRONI::GetNumberOfEdges();
  
    
  using LineGraphics = Graphics_2D::Arrow;
//   using LineGraphics = QGraphicsLineItem;
  QPen edgePen = QPen(edgeColor);
  edgePen.setWidthF(0);
  
  // add edges
  for( int i=0; i< num_edges; ++i)
  {
    VRONI::edge* vr_edge  = VRONI::edges+i;
    
    Geometry::Point_2D tail;
    get_node_coords(tail,vr_edge->n1);
    
    Geometry::Point_2D head;
    get_node_coords(head,vr_edge->n2);
   
    
    if( boost::geometry::distance(tail,head) > ZERO ) // NOTE: ZERO is a VRONI #define
    {
      auto line = new LineGraphics(tail.x(), tail.y(),
                                  head.x(), head.y());  
      
      line->setPen(edgePen);
      group->addToGroup(line);
    }
  }
  
  return group;
}

#endif // WITH_GRAPHICS


Simple2dGraph::Graph VroniInterface::createSiteShapeAssociationGraph()
{
  using Graph = Simple2dGraph::Graph;
  Graph graph;
  
  if( !m_vdIsCurrent )
  { 
    qDebug() << "Thr Voronoi diagram is not current. Can't create the site-to-shape association graph.";
    return graph;
  }
  
  
  auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
 
  QHash<int,Graph::vertex_descriptor> shapeVertexHash;
  
  // create the vertices representing the shapes (as they won't be itner connected)
  for( int i=0; i<shapeCentroidMap.size(); ++i)
  {
    Graph::vertex_descriptor shapeVertex = boost::add_vertex(graph);
        shapeVertexHash[i]=shapeVertex;
  
    vPosMap[shapeVertex] = shapeCentroidMap[i];
  }
  
  QHash<int,Graph::vertex_descriptor> segSiteHash;
  
  // create the vertices representing the SEG sites 
  for( int i=0; i<VRONI::num_segs; ++i)
  {
    Graph::vertex_descriptor segVertex = boost::add_vertex(graph);
    segSiteHash[i]=segVertex;
    
    VRONI::seg* vr_seg = VRONI::segs+i;
    
    Geometry::Point_2D tail;
    get_pnt_coords(tail,vr_seg->i1);
    
    Geometry::Point_2D head;
    get_pnt_coords(head,vr_seg->i2);
    
    vPosMap[segVertex] = Geometry::midpoint(head,tail);
    
    boost::add_edge(segVertex,shapeVertexHash[vr_seg->ext_appl],graph);
    
  }
    
  
  // get the PNT-to-shape map 
  QHash<int,int> pntShapeMap  = getPntToShapeMap();
    
  QHash<int,Graph::vertex_descriptor> pntSiteHash;  
  
  // create the vertices representing the PNT sites 
  for( int i=0; i<VRONI::num_pnts;++i )
  {
    Graph::vertex_descriptor pntVertex = boost::add_vertex(graph);
    pntSiteHash[i]=pntVertex;
    
    Geometry::Point_2D point;
    get_pnt_coords(point,i);
    
    vPosMap[pntVertex] = point;
    
    boost::add_edge(pntVertex,shapeVertexHash[pntShapeMap[i]],graph);
  }
  
  return graph;
    
}

Simple2dGraph::Graph VroniInterface::createEdgeSiteAssociationGraph()
{
  using Graph = Simple2dGraph::Graph;
  Graph graph;
  
  auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
 
  QHash<int,Graph::vertex_descriptor> pntSiteHash;
  QHash<int,Graph::vertex_descriptor> segSiteHash;
  QHash<int,Graph::vertex_descriptor> arcSiteHash;
  QHash<int,Graph::vertex_descriptor> vdEdgeHash;
  
  // first, create the vertices representing the sites (as they won't be connected)
  
  for( int i=0; i<VRONI::num_pnts;++i )
  {
    Graph::vertex_descriptor vertex = boost::add_vertex(graph);
    pntSiteHash[i]=vertex;
    
    Geometry::Point_2D point;
    get_pnt_coords(point,i);
    
    vPosMap[vertex] = point;
  }
  
  for( int i=0; i<VRONI::num_segs; ++i)
  {
    Graph::vertex_descriptor vertex = boost::add_vertex(graph);
    segSiteHash[i]=vertex;
    
    VRONI::seg* vr_seg = VRONI::segs+i;
    
    Geometry::Point_2D tail;
    get_pnt_coords(tail,vr_seg->i1);
    
    Geometry::Point_2D head;
    get_pnt_coords(head,vr_seg->i2);
    
    vPosMap[vertex] = Geometry::midpoint(head,tail);
  }
  
  for( int i=0; i<VRONI::num_arcs; ++i)
  {
    Graph::vertex_descriptor vertex = boost::add_vertex(graph);
    arcSiteHash[i]=vertex;
    
    VRONI::arc* vr_arc = VRONI::arcs+i;
    
    Geometry::Point_2D tail;
    get_pnt_coords(tail,vr_arc->i1);
    
    Geometry::Point_2D head;
    get_pnt_coords(head,vr_arc->i2);
    
    vPosMap[vertex] = Geometry::midpoint(head,tail);
  }
  
  auto inclusion_check = always_true;
  
  // second, create the vertices representing the edges and connect the resp. 
  // vertices
  for( int i=0; i<VRONI::GetNumberOfEdges(); ++i)
  {
    VRONI::edge* vr_edge = VRONI::edges+i;
    
    if( inclusion_check(vr_edge))
    {
      Graph::vertex_descriptor vertex = boost::add_vertex(graph);
      vdEdgeHash[i] = vertex;
      
      
      Geometry::Point_2D tail;
      get_node_coords(tail,vr_edge->n1);
      
      Geometry::Point_2D head;
      get_node_coords(head,vr_edge->n2);
      
      vPosMap[vertex] = Geometry::midpoint(head,tail);
      
      // get left site vertex
      
      Graph::vertex_descriptor left;
      switch(vr_edge->ltype)
      {
        case VRONI::PNT : left = pntSiteHash[vr_edge->lft]; break;
        case VRONI::SEG : left = segSiteHash[vr_edge->lft]; break;
        case VRONI::ARC : left = arcSiteHash[vr_edge->lft]; break;
        default: ;  // this shouldn't happen
      }
      boost::add_edge(left,vertex,graph);
      
      Graph::vertex_descriptor right;
      switch(vr_edge->rtype)
      {
        case VRONI::PNT : right = pntSiteHash[vr_edge->rgt]; break;
        case VRONI::SEG : right = segSiteHash[vr_edge->rgt]; break;
        case VRONI::ARC : right = arcSiteHash[vr_edge->rgt]; break;
        default: ;  // this shouldn't happen
      }
      boost::add_edge(right,vertex,graph);
    }
    
  }
  
  return graph;
  
}

Simple2dGraph::Graph VroniInterface::createEdgeShapeAssociationGraph()
{
  using Graph = Simple2dGraph::Graph;
  Graph graph;
  
  if( !m_vdIsCurrent )
  { 
    qDebug() << "Thr Voronoi diagram is not current. Can't create the edge-to-shape association graph.";
    return graph;
  }
  
  auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
 
  QHash<int,Graph::vertex_descriptor> shapeHash;
  QHash<int,Graph::vertex_descriptor> vdEdgeHash;
  
    
  // create the vertices representing the shapes (as they won't be connected)
  for( int i=0; i<shapeCentroidMap.size(); ++i)
  {
    Graph::vertex_descriptor shapeVertex = boost::add_vertex(graph);
    shapeHash[i]=shapeVertex;
  
    vPosMap[shapeVertex] = shapeCentroidMap[i];
  }
  
  // get the SITE-to-shape maps 
  QHash<int,int> pntToShapeMap  = getPntToShapeMap();
  QHash<int,int> segToShapeMap  = getSegToShapeMap();
  
  printVroniSiteData();
  qDebug() << "PNT-to-Shape Map:\n" <<  pntToShapeMap;
  qDebug() << "SEG-to-Shape Map:\n" <<  segToShapeMap;

  auto inclusion_check = in_w_mat;
  
  // third, create the vertices representing the edges and connect the resp. 
  // vertices
  for( int i_vr_edge=0; i_vr_edge<VRONI::GetNumberOfEdges(); ++i_vr_edge)
  {
    VRONI::edge* vr_edge = VRONI::edges+i_vr_edge;
    
    // check whether the current edge should be included in this extraction
    if( inclusion_check(vr_edge ) )
    {
//       QString edgeString = QString("Edge %1 :").arg(i_vr_edge,2);
      
      Graph::vertex_descriptor edgeVertex = boost::add_vertex(graph);
      vdEdgeHash[i_vr_edge] = edgeVertex;
      
      
      Geometry::Point_2D tail;
      get_node_coords(tail,vr_edge->n1);
      
      Geometry::Point_2D head;
      get_node_coords(head,vr_edge->n2);
      
      vPosMap[edgeVertex] = Geometry::midpoint(head,tail);
      
      // get left site index
            
      int leftShapeIndex = NIL;
      switch(vr_edge->ltype)
      {
        case VRONI::PNT : 
          // NOTE: the .contains check is necessary as operator[] default 
          // constructs an entry in case the key isn't present -- and that default
          // constucted int is not VRONI::NIL.
          if( pntToShapeMap.contains(vr_edge->lft) )
          { leftShapeIndex = pntToShapeMap[vr_edge->lft]; }
//           edgeString.append(QString("PNT %1").arg(vr_edge->lft,2));
          break;
        case VRONI::SEG :
          if( segToShapeMap.contains(vr_edge->lft) )
          { leftShapeIndex = segToShapeMap[vr_edge->lft]; }
//           edgeString.append(QString("SEG %1").arg(vr_edge->lft,2));
          break;
        default: ; // this shouldn' happen
      }
      
      if( leftShapeIndex != NIL )
      {
        Graph::vertex_descriptor leftShape = shapeHash[leftShapeIndex];
        if( leftShapeIndex!=0 ) // don't add edges to the boundary
        { boost::add_edge(leftShape,edgeVertex,graph); }
//         edgeString.append(QString(" (# %1)").arg(leftShapeIndex,2));
      }
//       else
//       { edgeString.append(QString("       ")); } // space for the alignment
      
      int rightShapeIndex = NIL;
      switch(vr_edge->rtype)
      {
        case VRONI::PNT : 
          // NOTE: the .contains check is necessary as operator[] default 
          // constructs an entry in case the key isn't present -- and that default
          // constucted int is not VRONI::NIL.
          if( pntToShapeMap.contains(vr_edge->rgt) )
          { rightShapeIndex = pntToShapeMap[vr_edge->rgt]; }
//           edgeString.append(QString("PNT %1").arg(vr_edge->rgt,2));
          break;
        case VRONI::SEG :
          if( segToShapeMap.contains(vr_edge->rgt) )
          { rightShapeIndex = segToShapeMap[vr_edge->rgt]; }
//           edgeString.append(QString("SEG %1").arg(vr_edge->rgt,2));
          break;
        default: ; // this shouldn' happen
      }
      
      
      if( rightShapeIndex != NIL )
      {
        Graph::vertex_descriptor rightShape = shapeHash[rightShapeIndex];
        if( rightShapeIndex != 0 ) // don't add edges to the boundary
        { boost::add_edge(rightShape,edgeVertex,graph); }
//         edgeString.append(QString(" (# %1)").arg(rightShapeIndex,2));
      }
//       else
//       { edgeString.append(QString("       ")); } // space for the alignment
        
//       qDebug() << edgeString;
    }
  }
  
  return graph;
}

Simple2dGraph::Graph VroniInterface::createEdgeShapeAssociationGraph(
  QHash<int,int> shapeToEdgeMap
)
{
  using Graph = Simple2dGraph::Graph;
  Graph graph;
  
//   auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
  
  QHash<int,Graph::vertex_descriptor> vdEdgeHash;
  auto inclusion_check = in_w_mat;
  
  // create the vertices representing the edges 
  for( int i_vr_edge=0; i_vr_edge<VRONI::GetNumberOfEdges(); ++i_vr_edge)
  {
    VRONI::edge* vr_edge = VRONI::edges+i_vr_edge;
    
    // check whether the current edge should be included in this extraction
    if( inclusion_check(vr_edge ) )
    {
      Graph::vertex_descriptor edgeVertex = boost::add_vertex(graph);
      vdEdgeHash[i_vr_edge] = edgeVertex;
      
      
      Geometry::Point_2D tail;
      get_node_coords(tail,vr_edge->n1);
      
      Geometry::Point_2D head;
      get_node_coords(head,vr_edge->n2);
      
      vPosMap[edgeVertex] = Geometry::midpoint(head,tail);
    }
  }
 
  QHash<int,Graph::vertex_descriptor> shapeHash;
  
  // create the vertices representing the shapes and connect them to the edges
  for( int i=0; i<shapeCentroidMap.size(); ++i)
  {
    
    // create vertex
    
    Graph::vertex_descriptor shapeVertex = boost::add_vertex(graph);
    shapeHash[i]=shapeVertex;
  
    vPosMap[shapeVertex] = shapeCentroidMap[i];
    
    // connect to edge(s)
    
    auto connectedEdges = shapeToEdgeMap.values(i);
    
    for( auto edge : connectedEdges)
    { 
      if( vdEdgeHash.contains(edge) )
      { boost::add_edge(shapeVertex,vdEdgeHash[edge],graph); }
    }
    
  }
  
  return graph;
}

void VroniInterface::computeVD(const VroniInterface::VDDTParameter& params) 
{
  // VRONI is not const correct, hence some const casting is necessary
  auto output_file = const_cast<char*>( params.site_file.c_str() );
  auto vd_dt_file  = const_cast<char*>( params.points_VDDT_file.c_str() );
  
  // call the VRONI function computing the Voronoi diagram and the Delaunay 
  // triangulation
  VRONI::API_ComputeVD(
    params.save_input, params.new_data, params.timing, params.bound, 
    params.sample,params.approx, output_file, params.check_for_duplicates, 
    params.recover_arcs, params.apx_absolute, params.apx_relative, 
    params.auto_apx, params.points_only, params.save_point_output, vd_dt_file, 
    params.clean_data
  );
  
  // mark the current VD/DT data as current
  m_vdIsCurrent = true;
}

/** \todo Maybe introduce an exception to indicate the out-of-dateness of the
 *    VD/DT data instead of implicitely calling computeVD()? This could 
 *    improve the const-correctness of the overall interface.
 */
void VroniInterface::computeWMAT(double WMAT_distance, double WMAT_angle, bool auto_WMAT, bool left, bool right, bool timing)
{
  // check if the VD/DT data is current. If not, update the VD/DT data
  if( !m_vdIsCurrent )
  { computeVD(); }
  
  // call VRONI's API function
  // NOTE: technically the function prototype calls for VRONI:boolean as bool
  //   input types. But since those are just typdef'ed to bool this should work.
  VRONI::API_ComputeWMAT(auto_WMAT, WMAT_angle,WMAT_distance,timing,left,right);
}

void VroniInterface::computeWMAT(const VroniInterface::WMATParameter& params) 
{
  // check if the VD/DT data is current. If not, update the VD/DT data
  if( !m_vdIsCurrent )
  { computeVD(); }
  
  // call VRONI's API function
  // NOTE: technically the function prototype calls for VRONI:boolean as bool
  //   input types. But since those are just typdef'ed to bool this should work.
  VRONI::API_ComputeWMAT(
    params.auto_WMAT, params.WMAT_angle, params.WMAT_distance, params.timing, 
    params.left, params.right
  );
}


VroniGraph::Graph VroniInterface::extractVdGraph(std::function< bool (VRONI::edge*)> inclusion_check) const
{
  using Graph = VroniGraph::Graph;
  Graph graph; ///< a new boost graph to hold the VD graph.
    
  auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
  auto vVroniMap  = boost::get(boost::vertex_vroniIndex, graph); ///< Vertex_VroniIndex_Map 
  auto vDataMap   = boost::get(boost::vertex_vroniNodeData, graph); ///<  Vertex_VroniNodeData_Map 
  auto eWeightMap = boost::get(boost::edge_weight, graph); ///<  Edge_Weight_Map   
  auto eVroniMap  = boost::get(boost::edge_vroniIndex, graph); ///<  Edge_VroniIndex_Map 
  auto eDataMap   = boost::get(boost::edge_vroniEdgeData, graph); ///<  Edge_VroniEdgeData_Map


  
  // A hash to relate VRONI node indices to the corresponding boost vertices
  QHash<int,Graph::vertex_descriptor> vroniIndexVertexHash;
  // A hash to relate VRONI edge indices to the corresponding boost edges
  QHash<int,Graph::edge_descriptor>   vroniIndexEdgeHash;

  // A counter to assign consecutive indices to the vertices (which will
  // differ from the VRONI indices stored in vVroniMap!)
  unsigned int vertexCounter = 0;
  
  
  auto get_vertex_descriptor = [&](Graph::vertex_descriptor & vertex, int vroniNodeIndex)
  {
    if( !vroniIndexVertexHash.contains(vroniNodeIndex) )
    { //no. add a new vertex.
      vertex = boost::add_vertex(graph);
      vroniIndexVertexHash[vroniNodeIndex]=vertex;
          
      { // populate the vertex properties
      
        vIndexMap[vertex]=vertexCounter++;
        vVroniMap[vertex]=vroniNodeIndex;
        
        VRONI::node* vroniNode = VRONI::nodes+vroniNodeIndex;
        vDataMap[vertex]=*vroniNode;
        
        Geometry::Point_2D position(vroniNode->p.x,vroniNode->p.y);
        if( VRONI::scale_data)
        { position = unScale(position); }
        vPosMap[vertex]=position;
        
        

      }
    }
    else
    { //yes. recover it.
      vertex = vroniIndexVertexHash.value(vroniNodeIndex);
    }
  };

    
  //
  // process the VRONI data and copy them over to the boost graph
  // ============================================================
  //
  
  
  // The total number of edges in VRONI's internal data. This includes VD/DT and
  // WMAT edges, but no data from the input site.
  int number_of_edges = VRONI::GetNumberOfEdges();

  // the index for the currently processed VRONI edge
  for(  int i_vr_edge = 4; // NOTE: I have NO IDEA why it has to start on 4, but this is a straight copy from VRONI::AddWMATToBuffer().
        i_vr_edge < number_of_edges;
        ++i_vr_edge)
  { // loop over all edges in the VRONI data, i.e. WMAT and VD/DT:
    
    // get a pointer to the currently processed edge
    VRONI::edge* vr_edge = VRONI::edges+i_vr_edge;
    
    // check whether the current edge should be included in this extraction
    if( inclusion_check(vr_edge ) )
    {
      Graph::vertex_descriptor tail;
            get_vertex_descriptor( tail, vr_edge->n1 );

      Graph::vertex_descriptor head;
            get_vertex_descriptor( head, vr_edge->n2 );
      
      {/* connect the head and the tail in the boost graph and compute the edge
        * length
        */
        using namespace boost;
        
        bool success= false;
        Graph::edge_descriptor edge;
        boost::tie(edge,success) = boost::add_edge(tail, head, graph);
        
        if(success)
        {
          eVroniMap[edge]   = i_vr_edge;
          eDataMap[edge]    = *vr_edge;
          eWeightMap[edge]  = geometry::distance(vPosMap[head],vPosMap[tail]);
          vroniIndexEdgeHash[i_vr_edge] = edge;
        }
      }
      
    }
  }
  
  // Initialize the interior edge index
  auto eIndexMap  = boost::get(boost::edge_index, graph); ///<  Edge_Index_Map 
  boost::graph_traits<Graph>::edges_size_type edge_count = 0;
  boost::graph_traits<Graph>::edge_iterator ei, ei_end;
  for(boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
  { eIndexMap[*ei] = edge_count++; }

  
  
  
  
  
  // done
  return graph;  
}



VroniGraph::Graph VroniInterface::extractDtGraph(void ) const
{
  using Graph = VroniGraph::Graph;
  
  if( !m_isPointSite )
  { 
    qDebug() << "The VRONI site isn't a pure point site. Cannot extract Delaunay triangulation and/or graph.";
    return Graph();
  }
  
  Graph graph; ///< a new boost graph to hold the DT graph.
  
  auto vIndexMap  = boost::get(boost::vertex_index,graph); ///< Vertex_Index_Map
  auto vPosMap    = boost::get(boost::vertex_position_2D, graph); ///< Vertex_Position2d_Map 
  auto vVroniMap  = boost::get(boost::vertex_vroniIndex, graph); ///< Vertex_VroniIndex_Map 
  auto vDataMap   = boost::get(boost::vertex_vroniNodeData, graph); ///<  Vertex_VroniNodeData_Map 
  auto eWeightMap = boost::get(boost::edge_weight, graph); ///<  Edge_Weight_Map   
  
  //NOTE: There is no VRONI data associated with these edges, as VRONI doesn't
  // explicitely compute them.
  //   auto eVroniMap  = boost::get(boost::edge_vroniIndex, graph); ///<  Edge_VroniIndex_Map 
  //   auto eDataMap   = boost::get(boost::edge_vroniEdgeData, graph); ///<  Edge_VroniEdgeData_Map
 
  // A hash to relate VRONI node indices to the corresponding boost vertices
  QHash<int,Graph::vertex_descriptor> vroniIndexHash;

  // A counter to assigne consecutive indices to the vertices (which will
  // differ from the VRONI indices stored in vVroniMap!)
  unsigned int vertexCounter = 0;
  
  // lambda to check if any of those VRONI nodes already is in the index hash
  auto get_site_vertex = [&](Graph::vertex_descriptor & vertex, int vroniNodeIndex)
  {
    if( !vroniIndexHash.contains(vroniNodeIndex) )
    { //no. add a new vertex.
      vertex = boost::add_vertex(graph);
      vroniIndexHash[vroniNodeIndex]=vertex;
          
      { // populate the vertex properties
      
        vIndexMap[vertex]=vertexCounter++;
        vVroniMap[vertex]=vroniNodeIndex;
        
        VRONI::node* vroniNode = VRONI::nodes+vroniNodeIndex;
        vDataMap[vertex]=*vroniNode;
        
        Geometry::Point_2D position(VRONI::pnts[vroniNodeIndex+2].p.x/*NOTE: this is different from vroniNode->p.x*/,
                                    VRONI::pnts[vroniNodeIndex+2].p.y/*NOTE: this is different from vroniNode->p.y*/);
        if( VRONI::scale_data)
        { position = unScale(position); }
        vPosMap[vertex]=position;

      }
    }
    else
    { //yes. recover it.
      vertex = vroniIndexHash.value(vroniNodeIndex);
    }
  };
  
  // get the Delaunay triangulation from VRONI
  std::vector<VRONI::d_triangle> dtTriangles = VRONI::GetDelaunayTriangles();
    
  for( VRONI::d_triangle& triangle : dtTriangles )
  {
    // get the involved nodes
    Graph::vertex_descriptor n1;
        get_site_vertex(n1,triangle.n1);
    Graph::vertex_descriptor n2;
        get_site_vertex(n2,triangle.n2);
    Graph::vertex_descriptor n3;
        get_site_vertex(n3,triangle.n3);

    // connect them in the boost graph and compute the edge lengths
    {
     
      bool success= false;
      Graph::edge_descriptor edge;
      
      boost::tie(edge,success) = boost::add_edge(n1, n2, graph);
      if(success)
      { 
        
        eWeightMap[edge] = boost::geometry::distance(vPosMap[n1],vPosMap[n2]); 
      }

      boost::tie(edge,success) = boost::add_edge(n2, n3, graph);
      if(success)
      { 
        
        eWeightMap[edge] = boost::geometry::distance(vPosMap[n2],vPosMap[n3]);
      }
      
      boost::tie(edge,success) = boost::add_edge(n3, n1, graph);
      if(success)
      { 
        
        eWeightMap[edge] = boost::geometry::distance(vPosMap[n3],vPosMap[n1]);
      }
      
    }
   
  }
  
//   VRONI::WriteVDDT("vddt.out");
  
  return graph;
  
}





VroniGraph::Graph VroniInterface::extractVdGraph(void ) const
{
  return extractVdGraph(always_true);
}
VroniGraph::Graph VroniInterface::extractWmatGraph(void ) const
{
  return extractVdGraph(in_w_mat);
}
VroniGraph::Graph VroniInterface::extractNonWmatGraph(void ) const
{
  return extractVdGraph(in_vddt);
}
 



QHash< int, int > VroniInterface::determineVoronoiCells()
{
  QHash<int,int> shapeToEdgeMap;
  
  if( !m_vdIsCurrent )
  { 
    qDebug() << "Thr Voronoi diagram is not current. Can't determine the Voronoi cells.";
    return shapeToEdgeMap;
  }
  
  // get the SITE-to-shape maps 
  QHash<int,int> pntToShapeMap  = getPntToShapeMap();
  QHash<int,int> segToShapeMap  = getSegToShapeMap();
  
  auto inclusion_check = in_w_mat;
  
  // third, create the vertices representing the edges and connect the resp. 
  // vertices
  for( int i_vr_edge=0; i_vr_edge<VRONI::GetNumberOfEdges(); ++i_vr_edge)
  {
    VRONI::edge* vr_edge = VRONI::edges+i_vr_edge;
    
    // check whether the current edge should be included in this extraction
    if( inclusion_check(vr_edge ) )
    {
            
      int leftShapeIndex = NIL;
      switch(vr_edge->ltype)
      {
        case VRONI::PNT : 
          // NOTE: the .contains check is necessary as operator[] default 
          // constructs an entry in case the key isn't present -- and that default
          // constucted int is not VRONI::NIL.
          if( pntToShapeMap.contains(vr_edge->lft) )
          { leftShapeIndex = pntToShapeMap[vr_edge->lft]; }
          break;
        case VRONI::SEG :
          if( segToShapeMap.contains(vr_edge->lft) )
          { leftShapeIndex = segToShapeMap[vr_edge->lft]; }
          break;
        default: ; // this shouldn' happen
      }
      
      if( leftShapeIndex != NIL )
      { shapeToEdgeMap.insertMulti(leftShapeIndex,i_vr_edge); }
      
      int rightShapeIndex = NIL;
      switch(vr_edge->rtype)
      {
        case VRONI::PNT : 
          // NOTE: the .contains check is necessary as operator[] default 
          // constructs an entry in case the key isn't present -- and that default
          // constucted int is not VRONI::NIL.
          if( pntToShapeMap.contains(vr_edge->rgt) )
          { rightShapeIndex = pntToShapeMap[vr_edge->rgt]; }
          break;
        case VRONI::SEG :
          if( segToShapeMap.contains(vr_edge->rgt) )
          { rightShapeIndex = segToShapeMap[vr_edge->rgt]; }
          break;
        default: ; // this shouldn' happen
      }
      
      if( rightShapeIndex != NIL and rightShapeIndex != 0)
      { shapeToEdgeMap.insertMulti(rightShapeIndex,i_vr_edge); }

    }
  }
  
  return shapeToEdgeMap;
}

// QHash< int, Geometry::Polygon_2D > VroniInterface::extractVoronoiCellGeometries() const
// {
//   QHash<int, Geometry::Polygon_2D> geometries;
//   
//   if( !m_vdIsCurrent )
//   {
//     qDebug() << "The VD is not current, cannot extract Voronoi cell geometries.";
//     return geometries;
//   }
//   
//   QHash<int,int>  shapeToEdgeMap = determineVoronoiCells;
//   QList<int>      shapeIndices = shapeToEdgeMap.uniqueKeys;
//   
//   for( auto shape_i : shapeIndices)
//   {
//     Geometry::MultiPoint_2D cellCorners;
//     
//     // get a list of all edges associacted with that shape.
//     QList<int> edgeIndices = shapeToEdgeMap.values(shape_i);
//     
//     for( auto edge_i : edgeIndices)
//     {
//       VRONI::edge* vr_edge = VRONI::edges+edge_i;
//       
//       Geometry::Point_2D tail;
//       get_node_coords(tail,vr_edge->n1);
//       boost::geometry::append(cellCorners,tail);
//       
//       Geometry::Point_2D head;
//       get_node_coords(tail,vr_edge->n2);
//       boost::geometry::append(cellCorners,head);
//     }
//     
//     Geometry::Polygon_2D cellGeometry;
//     boost::geometry::convex_hull(cellCorners,cellGeometry);
//     geometries[shape_i] = cellGeometry;
//   }
//   
//   return geometries;
// }


Geometry::Box_2D VroniInterface::boundingBox() const
{ 
  using namespace VRONI;
  
  if (!bbox_added) //NOTE: the bbox gets added in VRONI::AddDummyCorners(int bound)
    SetBoundingBox();
  
  if( data_scaled )
  {
    Geometry::Point_2D minCorner((bb_min.x/scale_factor)+shift.x,
                                 (bb_min.y/scale_factor)+shift.y);
    Geometry::Point_2D maxCorner((bb_max.x/scale_factor)+shift.x,
                                 (bb_max.y/scale_factor)+shift.y);
    
    return Geometry::Box_2D(minCorner,maxCorner);
  }
  else
  {
    Geometry::Point_2D minCorner((bb_min.x),
                                 (bb_min.y));
    Geometry::Point_2D maxCorner((bb_max.x),
                                 (bb_max.y));
    
    return Geometry::Box_2D(minCorner,maxCorner);
    
  }
}

Geometry::Point_2D VroniInterface::unScale(const Geometry::Point_2D& point) const
{
  if( VRONI::data_scaled )
    return Geometry::Point_2D(point.get<0>()/VRONI::scale_factor+VRONI::shift.x,
                              point.get<1>()/VRONI::scale_factor+VRONI::shift.y);
  else
    return point;
}


void VroniInterface::get_pnt_coords(Geometry::Point_2D& point, const int& pntIndex) const
{
  VRONI::pnt* vr_pnt = VRONI::pnts+pntIndex;
  point.set_x(vr_pnt->p.x);
  point.set_y(vr_pnt->p.y);
  if(VRONI::scale_data)
  { point = unScale(point); }
};

void VroniInterface::get_node_coords(Geometry::Point_2D& point, const int& nodeIndex) const
{
  VRONI::node* vr_node = VRONI::nodes+nodeIndex;
  point.set_x(vr_node->p.x);
  point.set_y(vr_node->p.y);
  if(VRONI::scale_data)
  { point = unScale(point); }
}


// void VroniInterface::unScale(Geometry::Point_2D& point) const
// {
//   point.get<0>() /= VRONI::scale_factor;
//   point.get<0>() += VRONI::shift.x;
//   point.get<1>() /= VRONI::scale_factor;
//   point.get<1>() += VRONI::shift.y;
// }



void VroniInterface::printVroniSiteData()
{
  qDebug() <<"VRONI Site Data:" 
    <<"\nNumber of PNTs :" << VRONI::num_pnts
    <<"\nNumber of SEGs :" << VRONI::num_segs
    <<"\nNumber or ARCs :" << VRONI::num_arcs;
    
  qDebug() << "VRONI PNT Data :";
  for( int i=0; i<VRONI::num_pnts; ++i)
  { 
    Geometry::Point_2D p;
    get_pnt_coords(p,i);
    qDebug() << QString("PNT %1:(%2,%3)").arg(i,2).arg(p.x()).arg(p.y());
  }
  
  qDebug() << "VRONI SEG Data :";
  for( int i=0; i<VRONI::num_segs; ++i)
  { 
    
    Geometry::Point_2D tail,head;
    VRONI::seg* vr_seg = VRONI::segs+i;
    get_pnt_coords(tail,vr_seg->i1);
    get_pnt_coords(head,vr_seg->i2);
    
//     qDebug() << QString("SEG %1: PNT %2 -> PNT %3 : (%4,%5) -> (%6,%7)")
//     .arg(i,2)
//     .arg(vr_seg->i1,2)
//     .arg(vr_seg->i2,2)
//     .arg(tail.x(),8,'f',2).arg(tail.y(),8,'f',2)
//     .arg(head.x(),8,'f',2).arg(head.y(),8,'f',2) ;
    qDebug() << QString("SEG %1 (%2): PNT %3 -> PNT %4 ")
    .arg(i,2)
    .arg(vr_seg->ext_appl,2)
    .arg(vr_seg->i1,2)
    .arg(vr_seg->i2,2);
  }
  
  
  
}

QHash< int, int > VroniInterface::getSegToShapeMap() const
{
  QHash<int,int> segToShapeMap;
  
  for(int i=0; i<VRONI::num_segs; ++i)
  {
    VRONI::seg* vr_seg = VRONI::segs+i;
    segToShapeMap[i] = vr_seg->ext_appl;
  }
  
  return segToShapeMap;
}



QHash< int, int > VroniInterface::getPntToShapeMap() const
{
  QHash<int,int> pntShapeMap;
  
  // loop over all segments, grab the end points and the assoc. shape and
  // put it in the pntShapeMap
  for( int i=0; i< VRONI::num_segs; ++i)
  {
    VRONI::seg* vr_seg = VRONI::segs+i;
    
    pntShapeMap[vr_seg->i1] = vr_seg->ext_appl; // tail
    pntShapeMap[vr_seg->i2] = vr_seg->ext_appl; // head
    
  }
  
  return pntShapeMap;
}


bool VroniInterface::isVdCurrent() const
{
  return m_vdIsCurrent;
}



