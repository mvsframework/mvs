/*
 *  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
 * 
 */


#include <boost/graph/subgraph.hpp>


namespace boost {

  /** \brief Clear a boost::subgraph in a DIY way.
   * \pre SubGraph must be a boost::subgraph. */
  template <typename SubGraph>
  void clearSubgraph(SubGraph& graph)
  {
    //TODO: Find some concept checking classes for SubGraph...
    
    
    using  ChildrenList = typename std::list<SubGraph*>;
    
    // taking care of all the children; this is copied from ~subgraph()
    {
      for(typename ChildrenList::iterator i = graph.m_children.begin();
       i != graph.m_children.end(); ++i)
      {
          delete *i;
      }
    }
    
    // dealing with this graph 
    { 
      graph.m_graph.clear(); 
    }
    
  };

  
} // end namespace boost