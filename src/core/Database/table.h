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

#ifndef TABLE_H
#define TABLE_H

#include <iostream>
#include <vector>
#include <unordered_map>


namespace Database {

/** \brief A Base Class to manage Database Tables.
 * To avoid too much confusion, use 
 * \code using FooBarTable = Table<ID,FooBar>; \endcode.
 * 
 * \note As the underlying data structure is an unorderd_map, the operator[]
 *   does \e create entries when the corresponding key is not present!
 * 
 * \todo Make this a "Q-Table", i.e. something that is comparabe to use with
 * the Qt ModelView framework...
 */
template<typename ID, typename RowData>
class Table : public std::unordered_map<ID,RowData>
{
  /** \brief Free function to deal with outputting the TableRowData to a stream. */
  friend std::ostream& operator<<(std::ostream& out, const Table<ID,RowData>& table)
  { 
    int i = 0;
    for(const typename std::unordered_map<ID,RowData>::value_type& row : table ) 
    {
      const auto id = row.first;
      const auto data = row.second;
       
      if( i>0 )
      { out << "\n"; }
      out << i++ << " | "<< id <<" || ";
      out << data;
    }   

    return out;
  };  
};


/** \brief A base for the data in Table rows.
 * This struct is mainly there to provide an operator<< so that printing
 * of Tables doesn't cause a compile error.
 * \sa Table\<ID,RowData\>::operator<<(std::ostream& out, const Table\<ID,RowData\>& table)
 */
struct TableRowData
{
  /** \brief Free function to deal with outputting the TableRowData to a stream.
   * \note This function does not print anything nor does it flush the stream!
   * \todo This printing takes some time and might happen in parrallel threads,
   *   leading to intermixed table output. Need to look into manually locking or
   *   synchronizing the output.
   *   Compare http://stackoverflow.com/questions/6374264/is-cout-synchronized-thread-safe
   */
  friend std::ostream& operator<<(std::ostream& out, const TableRowData& rowData)
  { return out; };

};


} // end namespace Database




#endif // TABLE_H
