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

#include "GustInterface/debuganderrormessages.h"

QString enumToQString(const GUST::DataLinkMessageId& enumValue)
{
  using namespace GUST;
  switch( enumValue )
  {
    case DATALINK_MESSAGE0            : return QObject::tr("M0");
    case DATALINK_MESSAGE1            : return QObject::tr("M1");
    case DATALINK_MESSAGE_FLIGHTPLAN  : return QObject::tr("Flightplan");
    case DATALINK_MESSAGE_RMTCMD      : return QObject::tr("Remote Command");
    case DATALINK_MESSAGE_TRAJECTORY  : return QObject::tr("Trajectory");
    case DATALINK_MESSAGE_UP0         : return QObject::tr("UP0");
    default : return QString("(Unkonwn DataLinkMessageId = %1)").arg(enumValue); //TODO: put this into tr()
  }
}

QString enumToQString(const GUST::AltitueMode& enumValue)
{
 using namespace GUST; 
 switch( enumValue )
  {
    case ALT_ABS : return QObject::tr("Absolute (wrt. Datum)");
    case ALT_AGL : return QObject::tr("Above Ground Level");
    default : return QString("(Unkonwn AltitueMode = %1)").arg(enumValue); //TODO: put this into tr()
  } 
}

QString enumToQString(const GUST::ButtonMap& enumValue)
{
 using namespace GUST;
 using namespace GUST;switch( enumValue )
  {
    case SafeOnButton :         return QObject::tr("Safe ON");
    case SafeOffButton :        return QObject::tr("Safe OFF");
    case WowUpButton :          return QObject::tr("Weight-on-Wheels UP");
    case ShiftButton :          return QObject::tr("Shift");
    case WowDownButton :        return QObject::tr("Weight-on-Wheels DOWN");
    case CrCtrlButton :         return QObject::tr("CrCtrl");
    case PeekLeftButton :       return QObject::tr("Peek Left");
    case PeekRightButton :      return QObject::tr("Peek Right");
    case GoFastButton :         return QObject::tr("Go Fast");
    case GpsDeniedButton2 :     return QObject::tr("GPS Denied 1");
    case GpsDeniedButton1 :     return QObject::tr("GPS Denied 2");
    case ManualOverrideButton : return QObject::tr("Manual Override");
    default : return QString("(Unkonwn ButtonMap = %1)").arg(enumValue); //TODO: put this into tr()
  } 
}

QString enumToQString(const GUST::HeadingMode& enumValue)
{
  using namespace GUST;
  switch( enumValue )
  {
    case HDG_CONST :      return QObject::tr("Constant");
    case HDG_VELOCITY :   return QObject::tr("Velocity");
    case HDG_POINTPOINT : return QObject::tr("Point to Point");
    case HDG_STICK :      return QObject::tr("Stick");
    default : return QString("(Unkonwn HeadingMode = %1)").arg(enumValue); //TODO: put this into tr()
  } 
}

QString enumToQString(const GUST::ManeuverType& enumValue)
{
  using namespace GUST;
  switch( enumValue )
  {
    case MAN_CUTCORNER :      return QObject::tr("Cut Corner");
    case MAN_FLYTHROUGH :     return QObject::tr("Fly Through");
    case MAN_STOPAT :         return QObject::tr("Stop At");
    case MAN_ETURN :          return QObject::tr("E-Turn");
    case MAN_LANDING :        return QObject::tr("Landing");
    case MAN_PIROUETTE :      return QObject::tr("Pirouette");
    case MAN_EXT :            return QObject::tr("Extern");
    case MAN_REPLAY :         return QObject::tr("Replay");
    case MAN_TRACK :          return QObject::tr("Track");
    case MAN_CHASE :          return QObject::tr("Chase");
    case MAN_FORMATION :      return QObject::tr("Formation");
    case MAN_TRACK2 :         return QObject::tr("Track 2");
    case MAN_REPEAT :         return QObject::tr("Repeat");
    case MAN_SLOWROLL :       return QObject::tr("Slow Roll");
    case MAN_TAKEOFF :        return QObject::tr("Takeoff");
    case MAN_TRACK3 :         return QObject::tr("Track 3");
    case MAN_SLUNGFORMATION : return QObject::tr("Slung Load Formation");
    case MAN_SLAM :           return QObject::tr("SLAM");
    case MAN_CLIMB :          return QObject::tr("Climb");
    case MAN_TRACK4 :         return QObject::tr("Track 4");
    case MAN_HELIFLIP :       return QObject::tr("Heli Flip");
    case MAN_FWD_TUMBLE :     return QObject::tr("Forward Tumble");
    case MAN_INTERCEPT :      return QObject::tr("Intercept");
    default : return QString("(Unkonwn ManeuverType = %1)").arg(enumValue); //TODO: put this into tr()
  }
}

QString enumToQString(const GUST::TrajectoryCommand& enumValue)
{
  using namespace GUST;
  switch( enumValue )
  {
    case TRAJECTORY_GET :             return QObject::tr("Get");
    case TRAJECTORY_GO :              return QObject::tr("Go");
    case TRAJECTORY_GO2 :             return QObject::tr("Go 2");
    case TRAJECTORY_GO_LC :           return QObject::tr("Go LC");
    case TRAJECTORY_GPSDENIED_GPSON : return QObject::tr("GPS Denied (GPS ON)");
    case TRAJECTORY_GPSDENIED_OFF :   return QObject::tr("GPS Denied OFF");
    case TRAJECTORY_GPSDENIED_ON :    return QObject::tr("GPS Denied ON");
    case TRAJECTORY_MOTOR_START :     return QObject::tr("Start Motor");
    case TRAJECTORY_MOTOR_STOP :      return QObject::tr("Stop Motor");
    case TRAJECTORY_PANICSTOP :       return QObject::tr("Panic Stop");
    case TRAJECTORY_STOP :            return QObject::tr("Stop");
    case TRAJECTORY_WOW_OFF :         return QObject::tr("Weight-on-Wheels OFF");
    case TRAJECTORY_WOW_ON :          return QObject::tr("Weight-on-Wheels ON");
    
    default : return QString("(Unkonwn TrajectoryCommand = %1)").arg(enumValue); //TODO: put this into tr()
  } 
}

QString enumToQString(const GUST::TrajectoryStatus& enumValue)
{
  using namespace GUST;
  switch( enumValue )
  {
    case TRAJ_STATUS_INITIAL :  return QObject::tr("Traj Initial");
    case TRAJ_STATUS_CRUISE :   return QObject::tr("Traj Cruise");
    case TRAJ_STATUS_STOP :     return QObject::tr("Traj Stop");
    
    default : return QString("(Unkonwn TrajectoryStatus = %1)").arg(enumValue); //TODO: put this into tr()
  } 
}

QString enumToQString ( const GUST::M1_Safemode_Status& enumValue )
{
  using namespace GUST;
  switch( enumValue )
  {
    case M1_Safemode_Status::go       : return QObject::tr("GO"); 
    case M1_Safemode_Status::stop     : return QObject::tr("STOP"); 
    case M1_Safemode_Status::go2      : return QObject::tr("GO 2"); 
    case M1_Safemode_Status::slow     : return QObject::tr("SLOW"); 
    case M1_Safemode_Status::gcas     : return QObject::tr("GCAS"); 
    case M1_Safemode_Status::holding  : return QObject::tr("(s)STOP"); 
    case M1_Safemode_Status::gcas2    : return QObject::tr("GCAS"); //NOTE: this cuplication of M1_SAFEMODE_GCAS follows the use in GUST's panel.c
    case M1_Safemode_Status::gcasNoe  : return QObject::tr("NOE"); 
    case M1_Safemode_Status::panic    : return QObject::tr("PANIC"); 
    case M1_Safemode_Status::avoid    : return QObject::tr("AVOID"); 
    
    default: return QString("(Unkown M1SafemodeStatus = %1)").arg(static_cast<int>(enumValue));
  }
}




QDebug operator<<(QDebug dbg, const GUST::datalinkMessageFlightPlan_ref & m)
{
  dbg.nospace() << "\n"
    << "Flight Plan Message {\n"
    << "  \\\\ Header (excerpt)\n"
    << "  messageId   = " << m.messageID << "\n"
    << "  messageSize = " << m.messageSize << "\n"
    << "  \\\\ Payload\n"
    << "  manIndex    = " << static_cast<int>(m.manIndex) << "\n"
    << "  lastIndex   = " << static_cast<int>(m.lastIndex) << "\n"
    << "  extra       = " << static_cast<int>(m.extra) << "\n"
    << "  lostComm    = " << static_cast<int>(m.lostComm) << "\n"
    << "  \\\\ "<<static_cast<int>(m.lastIndex)+1<<" waypoints to follow\n"
    << "}";
    
  for( int i=0; i <= m.lastIndex; ++i )
  {
    //FIXME: warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
    dbg << *reinterpret_cast<const GUST::datalinkMessageFlightPlanWaypoint_ref*>(m.waypoint[i]);
  }
  
  return dbg.space();
}

QDebug operator<<(QDebug dbg, const GUST::datalinkMessageFlightPlanWaypoint_ref& m)
{
  using namespace GUST;
  
  char hdgMode =  m.hdgAltMode ;
  hdgMode = hdgMode << 4;
  hdgMode = hdgMode >> 4;
  HeadingMode hm = static_cast<HeadingMode>( hdgMode );
  
  char altMode =  m.hdgAltMode;
  altMode = m.hdgAltMode >> 4;
  AltitueMode am = static_cast<AltitueMode>(altMode);
  
  dbg.nospace() << "\n"
    << "Waypoint {\n"
    << "  type       = " << enumToQString(static_cast<ManeuverType>(m.type)) << "\n"
//     << "  hdgAltMode = " << static_cast<int>(m.hdgAltMode) << "\n"
    << "  hdgAltMode = " << "["<< enumToQString(hm) <<", "<< enumToQString(am) << "]\n"
    << "  vnom       = " << static_cast<int>(m.vnom) << "\n"
    << "  anom       = " << static_cast<int>(m.anom) << "\n"
    << "  x          = ["<< m.x[0] << ", " << m.x[1]<< ", " << m.x[2] << "]\n"
    << "  psi        = " << m.psi << "\n"
    << "}";
  
  return dbg.space();
  
}