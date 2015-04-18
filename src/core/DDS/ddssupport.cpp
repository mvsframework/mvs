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

#include "ddssupport.h"

#include <iostream>

#include "nodeid.h"
#include <ndds/pres/pres_participant.h>

#include <QtCore/QDebug>

namespace DDS {

QStringList propertyToStringList ( const DDS_PropertySeq& propertySequence )
{
  QStringList list;
  DDS_Long length = propertySequence.length();
  for( DDS_Long i = 0; i<length; ++i )
  {
    list.append(QString(propertySequence[i].name));
    list.append(QString(propertySequence[i].value));
  }
  return list;
}
  
  
QHash< QString, QString > propertyToStringHash ( const DDS_PropertySeq& propertySequence )
{
  DDS_Long length = propertySequence.length();
  
  QHash<QString, QString> hash;
  hash.reserve(length);
  
  for( DDS_Long i = 0; i<length; ++i )
  {
   hash.insert( QString(propertySequence[i].name)  ,
                QString(propertySequence[i].value) ); 
  }
  
  return hash; 
}




void appendHashToProperty ( const QHash< QString, QString >& hash, DDS_PropertySeq& propertySequence )
{

  DDS_Long length = propertySequence.length();
  DDS_Long size = hash.size();
  
  propertySequence.ensure_length( length+size, length+2*size);
  
  DDS_Property_t property;
  for( auto it = hash.begin(); it != hash.end(); ++it)
  {
    std::string name = it.key().toStdString();
    std::string value = it.value().toStdString();
    
    qDebug() << QString("Property[%1] = %2").arg(QString::fromStdString(name)).arg(QString::fromStdString(value));
    
    property.name  = DDS_String_dup( name.c_str()   ); //TODO : who deletes this memory?
    property.value = DDS_String_dup( value.c_str() ); //TODO : who deletes this memory?
    property.propagate = true; // Propagate this property during discovery.
    
    propertySequence[length++] = property;   
  }
  
  return;
}

DDS_PropertySeq hashToProperty ( const QHash< QString, QString >& hash )
{
  DDS_PropertySeq propertySequence;
  
  appendHashToProperty(hash, propertySequence);
  
  return propertySequence;
}

QString enumToQString ( const DDS_ReturnCode_t& retcode )
{
  /** Taken from http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/group__DDSReturnTypesModule.html#DDSReturnTypesModule_std_retcodes */
  switch( retcode )
  {
    // Standard return codes
    case  DDS_RETCODE_OK :                    return QObject::tr("Successful return.");
    case  DDS_RETCODE_ERROR :                 return QObject::tr("Generic, unspecified error.");
    case  DDS_RETCODE_ILLEGAL_OPERATION :     return QObject::tr("The operation was called under improper circumstances.");
    case  DDS_RETCODE_ALREADY_DELETED   :     return QObject::tr("The object target of this operation has already been deleted.");
    case  DDS_RETCODE_BAD_PARAMETER :         return QObject::tr("Illegal parameter value.");
    case  DDS_RETCODE_UNSUPPORTED :           return QObject::tr("Unsupported operation. Can only returned by operations that are unsupported.");
    
    // extra return codes
    case  DDS_RETCODE_PRECONDITION_NOT_MET :  return QObject::tr("A pre-condition for the operation was not met.");
    case  DDS_RETCODE_OUT_OF_RESOURCES :      return QObject::tr("RTI Connext ran out of the resources needed to complete the operation.");
    case  DDS_RETCODE_NOT_ENABLED :           return QObject::tr("Operation invoked on a DDSEntity that is not yet enabled.");
    case  DDS_RETCODE_IMMUTABLE_POLICY :      return QObject::tr("Application attempted to modify an immutable QoS policy.");
    case  DDS_RETCODE_INCONSISTENT_POLICY :   return QObject::tr("Application specified a set of QoS policies that are not consistent with each other.");
    case  DDS_RETCODE_TIMEOUT :               return QObject::tr("The operation timed out.");
    case  DDS_RETCODE_NO_DATA :               return QObject::tr("Indicates a transient situation where the operation did not return any data but there is no inherent error.");
    default:                                  return QObject::tr("(Unknown return code, retcode = %1)").arg(retcode); 
  }
}

/** Taken from http://community.rti.com/comment/689#comment-689 */
DDS_GUID_t convertToGuid ( const DDS_InstanceHandle_t& instanceHandle )
{
  DDS_GUID_t guid;
  memcpy(guid.value,reinterpret_cast<DDS_Octet const *>(&instanceHandle ),16);
  return guid;
}

/** Taken from http://community.rti.com/comment/689#comment-689 */
DDS_GUID_t convertToGuid ( const DDS_BuiltinTopicKey_t& builtinTopicKey )
{
  DDS_GUID_t guid;
#if RTI_ENDIAN_LITTLE
  { /* Little Endian */
    memcpy(guid.value,reinterpret_cast<DDS_Octet const *>(&builtinTopicKey), 16);

  }
#else // RTI_ENDIAN_LITTLE
  { /* Big Endian */
    DDS_Octet const * topicKeyBuffer   = reinterpret_cast<DDS_Octet const *>(&builtinTopicKey);
    for ( uint i=0; i< 4; ++i ) {
        DDS_Octet* guidElement       = &(guid.value[i*4]);
        DDS_Octet const * keyBufferElement  = topicKeyBuffer + i*4;
        guidElement[0] = keyBufferElement[3];
        guidElement[1] = keyBufferElement[2];
        guidElement[2] = keyBufferElement[1];
        guidElement[3] = keyBufferElement[0];
    }
    
  }
#endif // RTI_ENDIAN_LITTLE
  return guid;
}



} // end namespace DDS


//############################################################################//
//                                                                            //
//  Supporting DDS_InstanceHandel_t as a QHash Key-type                       //
//                                                                            //
//############################################################################//


bool operator== ( const DDS_InstanceHandle_t& lhs, const DDS_InstanceHandle_t& rhs )
{
  if( lhs.isValid and rhs.isValid )
  { 
    if( lhs.keyHash.length == rhs.keyHash.length )
    {
      for( uint i=0; i< lhs.keyHash.length; ++i )
      {
        if( lhs.keyHash.value[i] != rhs.keyHash.value[i] )
        { return false; }
      }
      return true;
    }
  }
//   else
  return false; 
}



uint qHash ( const DDS_InstanceHandle_t& handle )
{
 QByteArray qba;
 for( uint i=0; i< handle.keyHash.length; ++i )
 {qba.append(static_cast<char>(handle.keyHash.value[i]));}
 
 return qHash(qba); 
}


//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//

QDebug operator<< ( QDebug dbg, const DDS_Time_t& time )
{
  dbg.nospace() << time.sec <<"."<< time.nanosec <<" [s]";
  return dbg.space();
}

QDebug operator<< ( QDebug dbg, const DDS_ReturnCode_t& retcode )
{
  dbg.nospace() << DDS::enumToQString(retcode);
  return dbg.space();
}

QDebug operator<< ( QDebug dbg, const DDS_BuiltinTopicKey_t& key )
{
//   auto data = key.value;
//   long d1 = data[0];
//   long d2 = data[1];
//   long d3 = data[2];
//   long d4 = data[3];
//   
//   QString hex = "{" ;
//   hex.append( QString::number(d1,16).toUpper().append("-"));
//   hex.append( QString::number(d2,16).toUpper().append("-"));
//   hex.append( QString::number(d3,16).toUpper().append("-"));
//   hex.append( QString::number(d4,16).toUpper().append("}"));
//     
//   dbg.nospace() << hex;
//   return dbg.space();
  return dbg << DDS::convertToGuid(key);
}

QDebug operator<< ( QDebug dbg, const DDS_InstanceHandle_t& handle )
{
  // a DDS_InstanceHandel_t is a char[16], i.e. a 16 byte or 128 bit entitiy, 
  // and, accorging to http://community.rti.com/forum-topic/get-info-about-participants-losts
  // convertable into a GUID
  
//   long value[16]= {0};
//   for(uint i=0; i<16; ++i)
//   { value[i] = static_cast<long>(handle.keyHash.value[i]); }
// 
//   QString hex = "{";
//   for(uint i=0; i<16; i +=4)
//   {
//     hex.append( QString::number(value[i+0],16).toUpper().append(":") );
//     hex.append( QString::number(value[i+1],16).toUpper().append(":") );
//     hex.append( QString::number(value[i+2],16).toUpper().append(":") );
//     hex.append( QString::number(value[i+3],16).toUpper().append("-") );
//   }
//   hex.remove(hex.length()-1,1).append("}"); // remove the last trailing "-" and add a "}"
//   
//   dbg.nospace() << hex;
//   return dbg.space();
  return dbg << DDS::convertToGuid(handle);
}

QDebug operator<< ( QDebug dbg, const DDS_GUID_t& guid )
{
  // a DDS_GUID_t is a char[16], i.e. a 16 byte or 128 bit entitiy, 
  // and, accorging to http://community.rti.com/forum-topic/get-info-about-participants-losts
  // convertable into a InstanceHandle
  
  long value[16]= {0};
  for(uint i=0; i<16; ++i)
  { value[i] = static_cast<long>(guid.value[i]); }

  QString hex = "{";
  for(uint i=0; i<16; i +=4)
  {
    hex.append( QString::number(value[i+0],16).toUpper().append(":") );
    hex.append( QString::number(value[i+1],16).toUpper().append(":") );
    hex.append( QString::number(value[i+2],16).toUpper().append(":") );
    hex.append( QString::number(value[i+3],16).toUpper().append("-") );
  }
  hex.remove(hex.length()-1,1).append("}"); // remove the last trailing "-"
  
  dbg.nospace() << hex; 
  return dbg.nospace();
}


std::ostream& operator<< ( std::ostream& out, const DDS_ReturnCode_t& retcode )
{
  return out << DDS::enumToQString(retcode).toStdString().c_str();
}
