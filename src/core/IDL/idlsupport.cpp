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

#include "idlsupport.h"

#include <QtCore/qglobal.h>

//############################################################################//
//                                                                            //
//  ENUM to QString conversion                                                 //
//                                                                            //
//############################################################################//


QString enumToQString(const IDL::Data::StateDescriptorId& stateId)
{
  using namespace IDL::Data;
  
  switch( stateId )
  {
    case STATE_ID_NOT_APPLICABLE:
      return QString(); // NULL string, \sa QString::isNull();
    
    case STATE_ID_I_RUNNING:          return QObject::tr("Running");
    case STATE_ID_I_RUNNING_REGION:   return QObject::tr("(Item Region)");
      
    case STATE_ID_N_STANDBY:          return QObject::tr("Standby");
    case STATE_ID_N_ACTIVE:           return QObject::tr("Active");
    case STATE_ID_N_ACTIVE_REGION:    return QObject::tr("(Node Region)");
    case STATE_ID_N_JOINING:          return QObject::tr("Joining");
    case STATE_ID_N_OPERATIONAL:      return QObject::tr("Operational");
    case STATE_ID_N_LEAVING:          return QObject::tr("Leaving");

    case STATE_ID_D_LINK_DOWN:        return QObject::tr("COM Down");
    case STATE_ID_D_LINK_UP:          return QObject::tr("Link Up");
    case STATE_ID_D_NO_LINK:          return QObject::tr("No Link");
      
    case STATE_ID_V_AUTONOMOUS      : return QObject::tr("Autonomous");
    case STATE_ID_V_MOVING_TO_ROUTE : return QObject::tr("Moving to RouteGraph");
    case STATE_ID_V_MOVING_TO_HOME  : return QObject::tr("Moving to Home");
    case STATE_ID_V_PILOTED         : return QObject::tr("Piloted");
    case STATE_ID_V_EXECUTING       : return QObject::tr("Executing External Commands");
    case STATE_ID_V_EXTERNAL_PLAN   : return QObject::tr("Executing External Flightplan");
    case STATE_ID_V_LOITER          : return QObject::tr("Loiter");
    case STATE_ID_V_JOYSTICK        : return QObject::tr("Joystick Control");
    case STATE_ID_V_LOST_LINK       : return QObject::tr("Executing Lost Link Procedure");
    case STATE_ID_V_MOVING_TO_CONTROLSTATION : return QObject::tr("Moving to ControlStation");
    case STATE_ID_V_DUAL_HOP_GUIDANCE : return QObject::tr("Dual Hop Guidance");
    case STATE_ID_V_PERIMETER_GUIDANCE : return QObject::tr("Perimeter Guidance");
    case STATE_ID_V_DYNAMIC_GUIDANCE : return QObject::tr("Dynamic Guidance");
   
//     case STATE_ID_CS_PASSIVE_MONITOR :  return QObject::tr("Passive Monitor");
//     case STATE_ID_CS_ACTIVE_CONTROL :   return QObject::tr("Active Control");
// 
//     case STATE_ID_U_REGION :          return QObject::tr("(UAV Region)"); 
//     case STATE_ID_U_AUTONOMOUS :      return QObject::tr("Autonomous");
//     case STATE_ID_U_REMOTE_PILOTED :  return QObject::tr("Remote Piloted");
//     case STATE_ID_U_MONITORING :      return QObject::tr("Monitoring");
// 
//     case STATE_ID_AA_MOVING_TO_GRAPH : return QObject::tr("Moving to Graph");
//     case STATE_ID_AA_MOVING_TO_HOME :  return QObject::tr("Moving to Home");
//     case STATE_ID_AA_AUTO_0 :          return QObject::tr("Auto 0");
//     case STATE_ID_AA_AUTO_1 :          return QObject::tr("Auto 1");
//     case STATE_ID_AA_AUTO_2 :          return QObject::tr("Auto 2");
//     
//     case STATE_ID_GUST_REGION :        return QObject::tr("(GUST Region)");
    case STATE_ID_GUST_ACTIVE          : return QObject::tr("GUST Interface Active");
    case STATE_ID_GUST_INACTIVE        : return QObject::tr("GUST Interface Inactive");
//     case STATE_ID_GUST_SAFEMODE_ON :   return QObject::tr("SafeMode ON");
//     case STATE_ID_GUST_SAFEMODE_OFF :  return QObject::tr("SafeMode OFF");   
    case STATE_ID_GUST_TRAJ_REGION     : return QObject::tr("(Trajectory Region)");
    case STATE_ID_GUST_TRAJ_INITIAL    : return QObject::tr("Trajectory INITIAL");
    case STATE_ID_GUST_TRAJ_CRUISE     : return QObject::tr("Trajectroy CRUISE");
    case STATE_ID_GUST_TRAJ_STOP       : return QObject::tr("Trajectory STOP");
    case STATE_ID_GUST_GUIDANCE_REGION : return QObject::tr("(Guidance Region)");
    case STATE_ID_GUST_GUIDANCE_GO     : return QObject::tr("Guidance GO");
    case STATE_ID_GUST_GUIDANCE_STOP   : return QObject::tr("Guidance STOP");
      
    default: return QObject::tr("(Unknown IDL::Data::StateDescriptorId = %1)").arg(stateId);
  }
}


QString enumToQString ( const IDL::Data::NodeType& nodeType )
{
  using namespace IDL::Data;
  
  /** \note
   * Do \e not use QObject::tr() to register these strings for translation as
   * they might be used for a reverse lookup!
   */
  
  switch( nodeType )
  {
    case NODE_TYPE_BASIC :    return QString("Basic");
    case NODE_TYPE_GCS :      return QString("GCS");
    case NODE_TYPE_VEHICLE :  return QString("Vehicle");
    case NODE_TYPE_UNKNOWN : //NOTE: fallthrough intended!
    default:                return QString("(Unknown)"); 
  };
}


bool operator==(const IDL::Data::Md5Hash& lhs, const IDL::Data::Md5Hash& rhs)
{
  if( lhs.length() != rhs.length() )
  { return false; }
  
  for( int i = 0; i < lhs.length() ; ++i )
  { 
    if( lhs[i]!=rhs[i] )
    { return false; }
  }
  
  return true;
}

bool operator!=(const IDL::Data::Md5Hash& lhs, const IDL::Data::Md5Hash& rhs)
{
  return !(lhs==rhs);
}



/** \todo Combine this function and enumToQString(IDL::Data::NodeType) somehow
 * to minimize the chance of error?
 */
IDL::Data::NodeType qStringToNodeTypeEnum ( QString const & typeName )
{
  if( typeName == enumToQString(IDL::Data::NODE_TYPE_BASIC) )
    return IDL::Data::NODE_TYPE_BASIC;
  if( typeName == enumToQString(IDL::Data::NODE_TYPE_GCS) ) 
    return IDL::Data::NODE_TYPE_GCS;
  if( typeName == enumToQString(IDL::Data::NODE_TYPE_VEHICLE) ) 
    return IDL::Data::NODE_TYPE_VEHICLE;

  // default
  return IDL::Data::NODE_TYPE_UNKNOWN;
}


QString enumToQString ( const IDL::Commands::CommandTypes & commandType )
{
  using namespace IDL::Commands;
  
  /** \note
   * Do \e not use QObject::tr() to register these strings for translation as
   * they might be used for a reverse lookup!
   */
  
  switch( commandType )
  {
    case NULL_COMMAND :               return QString("NULL Command");
    case N_JOIN_NETWORK :             return QString("Join Network");
    case N_LEAVE_NETWORK :            return QString("Leave Network");
    case V_LOITER :              return QString("Loiter");
//     case V_CONTINUE_MOTION :          return QString("Continue Motion");
    case V_ENABLE_JOYSTICK_CONTROL :  return QString("Enable Joystick Control");  
    case V_EXECUTE_EXTERNAL_PLAN :    return QString("Execute External Plan");
    default:                return QString("(Unknown)"); 
  };
}

//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//


std::ostream& operator<<(std::ostream& out, const IDL::Data::StateDescriptorId& enumValue)
{
 return out << enumToQString(enumValue).toStdString();
}


QDebug operator<<(QDebug dbg, const IDL::Data::StateDescriptorId& enumValue)
{
  dbg.nospace() << enumToQString(enumValue);
  return dbg.space();
}


std::ostream& operator<< ( std::ostream& out, const IDL::Commands::CommandTypes& enumValue )
{
  return out << enumToQString(enumValue).toStdString();
}

QDebug operator<< ( QDebug dbg, const IDL::Commands::CommandTypes& enumValue )
{
  dbg.nospace() << enumToQString(enumValue);
  return dbg.space();
}


std::ostream& operator<< (std::ostream& out , const IDL::Data::NodeType& enumValue)
{
  return out << enumToQString(enumValue).toStdString();
}


QDebug operator<< ( QDebug dbg , const IDL::Data::NodeType& enumValue)
{
  dbg.nospace() << enumToQString(enumValue);
  return dbg.space();
}

std::ostream& operator<< (std::ostream& out , IDL::Data::Position_Datum const & position)
{
  IDL::Data::Position_Datum const & p = position;
  return out << "("<< p.x<< ", " << p.y << ", " << p.z << ")";
}


QDebug operator<< ( QDebug dbg , IDL::Data::Position_Datum const & position)
{
  IDL::Data::Position_Datum const & p = position;
  dbg.nospace() << "("<< p.x<< ", " << p.y << ", " << p.z << ")";
  return dbg.space();
}



QByteArray IDL::toQByteArray(IDL::Data::Md5Hash md5Hash)
{
  char* start = reinterpret_cast<char*>(md5Hash.get_contiguous_buffer());
  int size = md5Hash.length()* sizeof(DDS_Octet);
  
  QByteArray ba(start,size);
  
  return ba;
}

IDL::Data::Md5Hash IDL::toMd5Hash(QByteArray byteArray)
{
  static const int MD5_HASH_SIZE = 16; // 128 bit
  Q_ASSERT( byteArray.size() == MD5_HASH_SIZE );
  
  IDL::Data::Md5Hash hash;
  hash.ensure_length(MD5_HASH_SIZE,MD5_HASH_SIZE);
  
  for(int i=0; i < byteArray.size(); ++i)
  { hash[i] = byteArray[i]; }
  
  return hash;
}




#include "idlsupport.moc"
