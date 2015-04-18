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

#ifndef PROPERTIES_H
#define PROPERTIES_H

/** \brief Amend the definitions the boost namespace
 * This is mainly used for graph properties.
 */
namespace boost {

//NOTE: these are the boos default properties
// (taken from Boost Graph Library, User Guide and Manual, §15.2.3)
//
//   enum vertex_index_t         { vertex_index          = 1 }; 
//   enum edge_index_t           { edge_index            = 2 }; 
//   enum edge_name_t            { edge_name             = 3 }; 
//   enum edge_weight_t          { edge_weight           = 4 }; 
//   enum vertex_name_t          { vertex_name           = 5 }; 
//   enum graph_name_t           { graph_name            = 6 }; 
//   enum vertex_distance_t      { vertex_distance       = 7 }; 
//   enum vertex_color_t         { vertex_color          = 8 }; 
//   enum vertex_degree_t        { vertex_degree         = 9 }; 
//   enum vertex_in_degree_t     { vertex_in_degree      = 10 }; 
//   enum vertex_out_degree_t    { vertex_out_degree     = 11 }; 
//   enum vertex_discover_time_t { vertex_discover_time  = 12 }; 
//   enum vertex_finish_time_t   { vertex_finish_time    = 13 }; 
  
  
  enum vertex_position_2D_t { vertex_position_2D  = 231 }; //NOTE: 231 is a totally random number, but it has to be unique in the program to allow boost to function.
  BOOST_INSTALL_PROPERTY(vertex, position_2D);
  
  enum vertex_id_t          { vertex_id           = 232 };
  BOOST_INSTALL_PROPERTY(vertex, id);

  enum vertex_locationId_t { vertex_locationId = 233 };
  BOOST_INSTALL_PROPERTY(vertex, locationId);
  
  //
  // VRONI specific properties
  //
  
  enum vertex_vroniIndex_t  { vertex_vroniIndex   = 234 };
  BOOST_INSTALL_PROPERTY(vertex, vroniIndex);
  
  enum edge_vroniIndex_t    { edge_vroniIndex     = 235 };
  BOOST_INSTALL_PROPERTY(edge, vroniIndex);
  
  enum vertex_vroniNodeData_t {vertex_vroniNodeData = 236 };
  BOOST_INSTALL_PROPERTY(vertex, vroniNodeData);
  
  enum edge_vroniEdgeData_t { edge_vroniEdgeData = 237 };
  BOOST_INSTALL_PROPERTY(edge, vroniEdgeData);
  
  //
  // COM graph properties
  //
  
  enum edge_quality_t { edge_quality = 237 };
  BOOST_INSTALL_PROPERTY(edge, quality);
  
  
  //
  // Obstacle graph properties
  //

  enum vertex_borderFlag_t { vertex_borderFlag = 238 };
  BOOST_INSTALL_PROPERTY(vertex, borderFlag);
  
}

#endif // PROPERTIES_H