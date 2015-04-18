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

#ifndef LOCATION_H
#define LOCATION_H

#include "locationid.h"
#include "Geometry/point_2d.h"

#include "Graph/routegraph.h"
#include "Graph/bifurcationgraph.h"
#include "Graph/obstaclegraph.h"



#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>

/** \brief A class to uniquely describe a location within in the environment.
 * \details The driving idea behind this class is to provide some glue between
 * the vertices of the different graphs (COM, route, visibility, etc.) and any
 * other element that can be associated with a position in the environemnt.
 * The (2D) position of the/a location can be thought of as the projection of 
 * a feature onto the environment plane.
 * \todo As the Location class has to be adapted anyways to function as a
 * boost::geometry::point, there is no real need to inherit from 
 * Geometry::Point_2D. As such, the relationshipd could be remodeled from a "is
 * a" to a "has a" -- which should/might/could be the more modular setup?
 */
class Location : public Geometry::Point_2D
{
public:
  /** \brief The ID type of a Location. */
  using Id = LocationId;
  
  using Geometry::Point_2D::set;
  using Geometry::Point_2D::get;

  /** \todo Maybe compute this based on the global position (lat, lon) so that
   * locations can be compared acros referece coordinate systems? 
   * \todo This is actually not unique as it depends on the endianness of the 
   * system used to compute the hash! */
  static Id computeId( Geometry::Point_2D const& position)
  { 
    double data [2] = { position.x(), position.y() };
    int length = sizeof data;
    return qHash(QByteArray::fromRawData(reinterpret_cast<char*>(&data[0]), length)); 
  };
  
  /** \brief Check if the provided ID is valid. */
  inline static bool isIdValid( Id const& id )
  { return !id==NULL_LOCATION_ID; }
  
  
  
public:
  /** \brief Default Constructor. */
  Location()
  :Location(Point_2D(), NULL_LOCATION_ID) {};
  
  /** \brief Constructor. */
  Location(Geometry::Point_2D const& position)
  : Location(position, computeId(position)) {};
  
  /** \brief Constructor. */
  Location(Geometry::Point_2D const& position, LocationId const& id)
  :Point_2D(position)
  ,locationId(id)
  {
    obstacleVertex = ObstacleGraph::Traits::null_vertex();
    routeVertex = RouteGraph::Traits::null_vertex();
    bifurcationVertex = BifurcationGraph::Traits::null_vertex();
  };
  
  /** \name RouteGraph Vertex Association */ //@{  
  void setRouteGraphVertex(RouteGraph::Vertex_Descriptor vertex)
  { routeVertex = vertex; };  
  RouteGraph::Vertex_Descriptor routeGraphVertex() const
  { 
    Q_ASSERT( isRouteGraphVertexValid() );
    return routeVertex;
  };
  bool isRouteGraphVertexValid() const
  { return routeVertex != RouteGraph::Traits::null_vertex(); }
  //@}
  
  /** \name BifurcationGraph Vertex Association */ //@{
  void setBifurcationGraphVertex(BifurcationGraph::Vertex_Descriptor vertex)
  { bifurcationVertex = vertex; };
  BifurcationGraph::Vertex_Descriptor bifurcationGraphVertex() const
  { 
    Q_ASSERT( isBifurcationGraphVertexValid() );
    return bifurcationVertex; 
  };
  bool isBifurcationGraphVertexValid() const
  { return bifurcationVertex != BifurcationGraph::Traits::null_vertex(); }
  //@}
  
  /** \name ObstacleGraph Vertex Association */ //@{
  void setObstacleGraphVertex(ObstacleGraph::Vertex_Descriptor vertex)
  { obstacleVertex = vertex; };
  ObstacleGraph::Vertex_Descriptor obstacleGraphVertex() const
  { 
    Q_ASSERT( isObstacleGraphVertexValid() );
    return obstacleVertex;
  };
  bool isObstacleGraphVertexValid() const
  { return obstacleVertex != ObstacleGraph::Traits::null_vertex(); };
  //@}
  
  
  /** \brief Get the LocationId of this location.
   * \details (This Id can also be used as a QHash key.) */
  Id id() const 
  { return locationId; }
 
private:
  /** \brief A unique identifier of this Location, created from a hash of the 
   * position. */
  Id locationId;
  
  ObstacleGraph::Vertex_Descriptor obstacleVertex;
  RouteGraph::Vertex_Descriptor routeVertex;
  BifurcationGraph::Vertex_Descriptor bifurcationVertex;
  
  
  
  
};

inline bool operator==(Location const& lhs, Location const& rhs)
{ return lhs.id()==rhs.id(); }

inline bool operator!=(Location const& lhs, Location const& rhs)
{ return !(lhs==rhs); };


// inline unsigned int qHash(Location const & location, unsigned int seed = 0)
// { return qHash(location.id()); };

//
// Adapt the Location to the boost::geometry point concept
// =======================================================
// This section provides the glue to properly make Point_2D work with boost
// by providing all the required traits and redirection functions.
//

namespace boost{ namespace geometry { namespace traits {
  
  template<>
  struct tag<Location>
  { typedef point_tag type; };
  
  template<>
  struct coordinate_type<Location>
  { typedef double type; };
  
  template<>
  struct coordinate_system<Location>
  { typedef boost::geometry::cs::cartesian type; };
  
  template<>
  struct dimension<Location>
    : boost::mpl::int_<2>{};
    
  template<std::size_t Dimension>
  struct access<Location, Dimension>
  {
    static inline double get(const Location& p)
    { return p.template get<Dimension>(); };
    
    static inline void set(Location& p, const double& value)
    { p.template set<Dimension>(value); };
  };
  
}}} // end namespave boost::geometry::traits



//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

inline QDebug operator<<(QDebug dbg, const Location& l )
{
  dbg.nospace()
    << "\nLOCATION"
    << "\n  id: " << l.id()
    << "\n  position: " << *static_cast<Geometry::Point_2D const*>(&l);
    
  dbg.nospace()
    << "\n  obstacle graph vertex: ";
    if( l.isObstacleGraphVertexValid() )
    { dbg.nospace() << l.obstacleGraphVertex(); }
    else
    { dbg.nospace() << "(invalid)"; }
    
  dbg.nospace()
    << "\n  route graph vertex: ";
    if( l.isRouteGraphVertexValid() )
    { dbg.nospace() << l.routeGraphVertex(); }
    else
    { dbg.nospace() << "(invalid)"; }
    
  dbg.nospace()
    << "\n  bifurcation graph vertex: ";
    if( l.isBifurcationGraphVertexValid() )
    { dbg.nospace() << l.bifurcationGraphVertex(); }
    else
    { dbg.nospace() << "(invalid)"; }
      
  return dbg.space();
};

#endif // LOCATION_H
