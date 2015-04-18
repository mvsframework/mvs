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

#ifndef DDSSUPPORT_H
#define DDSSUPPORT_H

#include <assert.h>
#include <QtCore/QDebug>
#include <QtCore/QHash>
#include <QtCore/QStringList>
#include <ndds/ndds_cpp.h>

namespace DDS {

QStringList             propertyToStringList( DDS_PropertySeq const & propertySequence );
QHash<QString, QString> propertyToStringHash( DDS_PropertySeq const & propertySequence );

void appendHashToProperty( QHash<QString,QString> const & hash, DDS_PropertySeq & propertySequence);

DDS_PropertySeq         hashToProperty( QHash<QString,QString> const & hash);


template<typename Sequence,
         typename Element>
void appendToSeq(Sequence & sequence ,Element const & element)
{
  DDS_Long currentLength  = sequence.length();
  DDS_Long currentMaximum = sequence.maximum();
  bool success = sequence.ensure_length(currentLength+1,currentMaximum*2);
  Q_ASSERT( success );
  
  sequence[currentLength] = element;  
};

DDS_GUID_t convertToGuid(DDS_InstanceHandle_t const & handle);
DDS_GUID_t convertToGuid( const DDS_BuiltinTopicKey_t& builtinTopicKey );


QString enumToQString(DDS_ReturnCode_t const & retcode);


} // end namespace DDS

//############################################################################//
//                                                                            //
//  DDS_GUID_t operators                                                      //
//                                                                            //
//############################################################################//
inline bool operator==(DDS_GUID_t const & lhs, DDS_GUID_t const & rhs)
{ 
  /** http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/group__DDSGUIDSupportModule.html# */
  return DDS_BOOLEAN_TRUE == DDS_GUID_equals(&lhs,&rhs);
};

inline bool operator!=(DDS_GUID_t const & lhs, DDS_GUID_t const & rhs){return !operator==(lhs,rhs);};

inline bool operator< (DDS_GUID_t const & lhs, DDS_GUID_t const & rhs)
{ 
  /** http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/group__DDSGUIDSupportModule.html# */
  return DDS_GUID_compare(&lhs,&rhs) < 0;
};

inline bool operator> (DDS_GUID_t const & lhs, DDS_GUID_t const & rhs){return  operator< (rhs,lhs);};
inline bool operator<=(DDS_GUID_t const & lhs, DDS_GUID_t const & rhs){return !operator> (lhs,rhs);};
inline bool operator>=(DDS_GUID_t const & lhs, DDS_GUID_t const & rhs){return !operator< (lhs,rhs);};

//############################################################################//
//                                                                            //
//  Supporting DDS_InstanceHandel_t as a QHash Key-type                       //
//                                                                            //
//############################################################################//
Q_DECLARE_METATYPE( DDS_InstanceHandle_t )
bool operator==(DDS_InstanceHandle_t const & lhs, DDS_InstanceHandle_t const & rhs);
uint qHash(DDS_InstanceHandle_t const & handle);




//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

QDebug operator<<(QDebug dbg, DDS_Time_t const & time);
QDebug operator<<(QDebug dbg, DDS_BuiltinTopicKey_t const & key);
QDebug operator<<(QDebug dbg, DDS_InstanceHandle_t const & handle);
QDebug operator<<(QDebug dbg, DDS_GUID_t const & guid);



QDebug operator<<(QDebug dbg, DDS_ReturnCode_t const & retcode);
std::ostream& operator<<(std::ostream& out, DDS_ReturnCode_t const & retcode);

#endif // DDSSUPPORT_H
