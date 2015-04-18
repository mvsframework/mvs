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

#ifndef NODEID_128_H
#define NODEID_128_H


#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/graph/buffer_concepts.hpp>
#include <assert.h>

#include <QtCore/QDebug>
#include <QtCore/QByteArray>

#include "IDL/data.h"
#include "DDS/ddssupport.h"





/** \brief A class holding Node specific 128 UUID data.
 * 
 * \todo missing bool opertor==(IDL::Data::NodeId, NodeId_128);
 */
class NodeId_128 : public boost::uuids::uuid
{
public:
  
  /** \brief Default (empty) constructor. */
  NodeId_128()
    :boost::uuids::uuid({{0}}){ };
  
  /** \brief Constructor. */
  NodeId_128(boost::uuids::uuid const & u)
    :boost::uuids::uuid(u) {};
  
  /** \brief Constructor. */
  NodeId_128(IDL::Data::NodeId const & idlId)
  { 
    Q_ASSERT( size()==static_cast<size_t>( idlId.length() ) );
    memcpy(data,idlId.get_contiguous_buffer(),size());
  };
  
  /** \brief Constructor. */  
  explicit NodeId_128(DDS_GUID_t const & guid)
  { 
    Q_ASSERT( size()==sizeof(guid.value) );
    memcpy(data,guid.value, sizeof(guid.value));
  };
  
  /** \brief Constructor. */
  explicit NodeId_128(DDS_InstanceHandle_t const & handle)
    :NodeId_128(DDS::convertToGuid(handle))
  {};
  
  /** \brief Constructor. */
  explicit NodeId_128(DDS_BuiltinTopicKey_t const & key)
    :NodeId_128(DDS::convertToGuid(key))
  {};


  explicit NodeId_128(QByteArray const & byteArray)
    :boost::uuids::uuid({{0}})
  {
    Q_ASSERT(byteArray.size() == static_cast<int>(size()));
    for( std::size_t i=0; i<size(); ++i)
    { data[i] = byteArray.at(i); }
  };
  
  
  DDS_GUID_t toDdsGuid() const
  {
    DDS_GUID_t guid;
    Q_ASSERT( size()==sizeof(guid.value) );
    memcpy(guid.value,data,size());
    return guid;
  };
  
  DDS_InstanceHandle_t toDdsInstanceHandle() const
  {
    DDS_InstanceHandle_t handle;
    memcpy(&(handle),&data,size());
    handle.isValid = DDS_BOOLEAN_TRUE;
    return handle;
  };
  
  IDL::Data::NodeId toIdlNodeId() const
  {
    DDS_Long length = static_cast<DDS_Long>(this->size());
    IDL::Data::NodeId idlId(length);
    
    //NOTE: If this assert fails, a common mistake is that the DDS sequence
    // for the IDL::Data:NodeId hasn't been properly initialized. Maybe a `new`
    // instead of a `TopicSupport::create_data()` was used?
    Q_ASSERT( idlId.maximum() == static_cast<DDS_Long>( this->size() ));
    
    idlId.from_array( this->data,this->size() );
    return idlId;
  };
  
  QByteArray toByteArray() const 
  {
    QByteArray qba;
    for(auto& byte : data)
    { qba.append(byte); }
    return qba;
  };
  
  std::string toString() const
  {
    return boost::uuids::to_string(*this);
  };
  
  QString toQString() const
  {
    return QString(toString().c_str());
  };
  
  
  NodeId_128& operator=(NodeId_128 const & rhs) = default;
    
  /** \brief Conversion operator to boost::uuid. 
   * \internal The conversion operators are taken from 
   * http://www.boost.org/doc/libs/1_53_0/libs/uuid/uuid.html .
   * \todo I honestly have no idea why they return a cast to a \e reference, 
   * but I assume that has something to do with boost::uuid being a base class.
   */
  operator boost::uuids::uuid()
  { return static_cast<boost::uuids::uuid&>(*this); }

  /** \brief Conversion operator to boost::uuid. */
  operator boost::uuids::uuid() const
  { return static_cast<boost::uuids::uuid const&>(*this); };

//   /** \brief Conversion operator to std::string. */
//   operator std::string() const
//   { return boost::uuids::to_string(*this); };
  
//   /** \brief Conversion operator to QString. */
//   operator QString() const
//   { return QString(std::string(*this).c_str()); };

//   /** \brief Conversion operator to DDS_GUID_t. */  
//   operator DDS_GUID_t() const
//   { return toDdsGuid(); };
  
 
//   /** \brief Conversion operator to DDS_InstanceHandle_t. */
//   operator DDS_InstanceHandle_t() const
//   { return toDdsInstanceHandle(); };
  
  /** \brief Conversion operator to IDL::Data::ID. */
  operator IDL::Data::NodeId() const
  { return toIdlNodeId(); };
  
  /** \brief Conversion operator to QByteArray. */
  operator QByteArray() const
  { return toByteArray(); }
  
protected:
  template<typename T1, typename T2, typename T3, typename T4>
  NodeId_128(T1 const & d1, T2 const & d2, T3 const & d3, T4 const & d4)
  {
    auto& raw_data = data;
 
    const size_t s1 = sizeof(d1);
    const size_t s2 = sizeof(d2);
    const size_t s3 = sizeof(d3);
    const size_t s4 = sizeof(d4);
    Q_ASSERT(s1+s2+s3+s4 == sizeof(raw_data));
       
    // fill raw_uuid_data
    memcpy( &raw_data[0], &d1, s1);
    memcpy( &raw_data[(s1)/sizeof(raw_data[0])], &d2, s2);
    memcpy( &raw_data[(s1+s2)/sizeof(raw_data[0])], &d3, s3);
    memcpy( &raw_data[(s1+s2+s3)/sizeof(raw_data[0])], &d4, s4);
  };
  
};


const NodeId_128 NULL_NODE_128_ID = NodeId_128();





namespace std {
  
  template<>
  struct hash<NodeId_128>
  {
    size_t operator()(const NodeId_128& u) const
    { return boost::uuids::hash_value(u); }
  };


  template<>
  inline void swap(NodeId_128& lhs, NodeId_128& rhs)
  { boost::uuids::swap(lhs,rhs); };


} // end namespace std

// inline unsigned int qHash(NodeId_128 const & u)
// { return boost::uuids::hash_value(u); };

//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

inline std::ostream& operator<<(std::ostream& out, const NodeId_128& uuid)
{ return out << boost::uuids::to_string(uuid); };

inline QDebug operator<<(QDebug dbg, const NodeId_128& uuid)
{ 
  dbg.nospace() << boost::uuids::to_string(uuid).c_str();
  return dbg.space();
};

inline QDataStream & operator<<(QDataStream &out, const NodeId_128 & id)
{
  out << id.toByteArray();
  out.setVersion(QDataStream::Qt_5_0);
  
  return out;
};

inline QDataStream & operator>>(QDataStream &in, NodeId_128 & id)
{
  if( in.version() == QDataStream::Qt_5_0 )
  {    
    QByteArray qba;
    in >> qba;
    id = NodeId_128(qba);
  }
  else
  {
   qCritical("Wrong QDataStream version. QDataStream::Qt_5_0 is required.");
  }
  
  return in;
};


#endif // NODEID_128_H
