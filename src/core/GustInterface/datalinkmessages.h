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

#ifndef DATALINKMESSAGES_H
#define DATALINKMESSAGES_H

#include <cstdio>
#include <climits>
#include <string>

namespace GUST {

/** \brief GUST datalink message identifyer. */  
enum DataLinkMessageId {
  DATALINK_MESSAGE0           =  0, ///< \sa GUST::datalinkMessage0_ref
  DATALINK_MESSAGE1           =  1, ///< \sa GUST::datalinkMessage1_ref
  DATALINK_MESSAGE_TRAJECTORY = 10, ///< \sa GUST::datalinkMessageTrajectory_ref
  DATALINK_MESSAGE_FLIGHTPLAN = 11, ///< \sa GUST::datalinkMessageFlightPlan_ref
  DATALINK_MESSAGE_UP0        = 12, ///< \sa GUST::datalinkMessageUp0_ref
  DATALINK_MESSAGE_RMTCMD     = 59  ///< \sa GUST::datalinkMessageRmtCmd_ref
};

/** \brief Synchronisation bytes for the header of a message.
 * The sync bytes--or rather the a priori knowledge of them--allows GUST to
 * seek through a sequence of bytes received and held in the receiving buffer 
 * and to determine where a new message starts.
 */
enum SyncBytes {
  SYNC_BYTE_1 = 0xa3, ///< 0xA3 = 163
  SYNC_BYTE_2 = 0xb2, ///< 0xB2 = 178
  SYNC_BYTE_3 = 0xc1  ///< 0xC1 = 193
};

/** \brief The type of maneuver executed at a particular waypoint. */
enum ManeuverType {
   MAN_CUTCORNER      =  0
  ,MAN_FLYTHROUGH     =  1
  ,MAN_STOPAT         =  2
  ,MAN_ETURN          =  3 ///< \brief Reverse course.
                           /// A 180 degree heading change in the shortes horizontal distance.
  ,MAN_LANDING        =  4
  ,MAN_PIROUETTE      =  5
  ,MAN_EXT            =  6
  ,MAN_REPLAY         =  7
  ,MAN_TRACK          =  8
  ,MAN_CHASE          =  9
  ,MAN_FORMATION      = 10
  ,MAN_TRACK2         = 14
  ,MAN_REPEAT         = 15
  ,MAN_SLOWROLL       = 16
  ,MAN_TAKEOFF        = 17
  ,MAN_TRACK3         = 18
  ,MAN_SLUNGFORMATION = 19
  ,MAN_SLAM           = 20
  ,MAN_CLIMB          = 21
  ,MAN_TRACK4         = 22 
  ,MAN_HELIFLIP       = 23
  ,MAN_FWD_TUMBLE     = 24
  ,MAN_INTERCEPT      = 25
};

enum HeadingMode {
   HDG_CONST        = 0 ///< \brief Point to a given heading.
  ,HDG_VELOCITY     = 1 ///< \brief Point forward, i.e. align with velocity vector.
  ,HDG_POINTPOINT   = 2 ///< \brief Point to a point on the ground (green cross).
  ,HDG_STICK        = 3
};

enum AltitueMode {
  ALT_ABS = 0,  ///< \brief Absolute altide wrt. to the (GUST) datum, i.e.in -Z or -"down" direction.
  ALT_AGL = 1   ///< \brief Altitude above ground level.
};
  
enum TrajectoryCommand {
  TRAJECTORY_STOP             = 0, ///< \brief Stop within the current acceleration and jerk limits.
                                   ///<  The stop will happen somewhere "in front" of the vehicle, at the current altitude.     
  TRAJECTORY_GO               = 1, ///< \brief Execute the currently uploaded flight plan.
  TRAJECTORY_GO2              = 2,
  TRAJECTORY_MOTOR_START      = 5,
  TRAJECTORY_MOTOR_STOP       = 6,
  TRAJECTORY_GET              = 7,
  TRAJECTORY_GO_LC            = 8,
  TRAJECTORY_PANICSTOP        = 9, ///< \brief Fullstop in the shortest horizontal distance (A/C will most likely climb.)
  TRAJECTORY_WOW_OFF          = 10,
  TRAJECTORY_WOW_ON           = 11,
  TRAJECTORY_GPSDENIED_ON     = 12,
  TRAJECTORY_GPSDENIED_GPSON  = 13,
  TRAJECTORY_GPSDENIED_OFF    = 14
};

/** \brief The status of the current trajectory/flightplan execution
 * The status of a trajectory execution is used in the GUST planner to aid
 * the trajectory generation. It is also stored and the current status is always
 * transmitted in datalinkMessage1_ref.traj_status.
 */
enum TrajectoryStatus {
  TRAJ_STATUS_INITIAL  = 0, ///<  The A/C is not in motion and holding position.
  TRAJ_STATUS_CRUISE   = 1, ///<  The A/C is currently executing a flight plan and following a trajectory.
  TRAJ_STATUS_STOP     = 2  ///<  The A/C has just reached the end of a flight plan and stopps. (Followed by status TRAJECTORY_INITIAL)
};

/** \brief The most important information about the current state of the guidance system.
 * \note Safemode in m1 is different from trajectory.safemode .
 * 
 * The <tt>safemode</tt> flag in the m1 message is basically hijacked to be the
 * overall state of the trajectory generator as displayed on the panel. It 
 * basically just tells the user the most important thing about the state of the
 * guidance system. 
 * 
 * \internal
 * 0: "executing a flightplan" (from navigation.cpp, line 3381) (*1)
 * 1: "hover at current position/heading, pause trajectory clock" (from controller.db, line 107)
 * 2: (*2)
 * 3: "trajSlow" (from datalink.cpp, line 3786)
 * 4: "gcas traj mod in progress" (from datalink.cpp, line 3780
 * 5: "slow holding pattern" (from datalink.cpp, line 3783)
 * 6: "gcas avoidance in progress" (from datalink.cpp, line 3788)
 * 7: "gcas avoidance in progress" (from datalink.cpp, line 3790)
 * 8: panic stop (from datalink.cpp, line 3777)
 * 9: "collision avoidance" (from: datalink.cpp, line 3776)
 * (*1):trajGo seems to engage this mode (from datalink.cpp, line 1481)
 * (*2):trajGo2 seems to engage this mode (from datalink.cpp, line 1509)
 * 
 */
enum M1_Safemode_Status {
   go       = 0 ///< \brief Executing a flightplan.
  ,stop     = 1 ///< \brief Hovering at current position/heading.
  ,go2      = 2 ///< \brief Exectuing a flightplan.
  ,slow     = 3 ///< \brief Slow trajectory.
  ,gcas     = 4 ///< \brief Ground collision avoidance.
  ,holding  = 5 ///< \brief Slow holding pattern.
  ,gcas2    = 6 ///< \brief Ground collision avoidance.
  ,gcasNoe  = 7 ///< \brief Nap-of-the-Earth (ground collision avoidance).
  ,panic    = 8 ///< \brief Panic Stop.
  ,avoid    = 9 ///< \brief Collision Avoidance.
};



/** \brief The common header structure of all GUST datalink messages.
 * This class is mainly provided to be able to utilize
 * sizeof(datalinkHeader_ref) in order to compute the byte size of message 
 * headers.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message.
 */
struct datalinkHeader_ref {
  datalinkHeader_ref()
    :messageID(DATALINK_MESSAGE0)
    ,messageSize(sizeof(datalinkHeader_ref))
    ,hcsum(0)
    ,csum(0)
  { };    
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare = 0;///< \brief Spare (sync) byte.
  int messageID; ///< \brief Message identifier.
  int messageSize;///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum; ///< \brief Modified Fletcher32 checksum of the complete message.
} __attribute__((packed)); 



/** \brief The (LEGACY) common header structure of all GUST datalink messages.
 * This class is mainly provided to be able to utilize
 * sizeof(datalinkHeaderOld_ref) in order to compute the byte size of message 
 * headers.
 * 
 * \deprecated
 */
struct datalinkHeaderOld_ref {
  unsigned char sync1; /*  */
  unsigned char sync2; /*  */
  unsigned char hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned char csum; ///< \brief Modified Fletcher32 checksum of the complete message.
  int messageID; ///< \brief Message identifier.
  int messageSize; ///< \brief Total byte size of this message, incl. header.
}__attribute__((packed));



/** \brief Message struct for general status information of the A/C. 
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessage0_ref {
  datalinkMessage0_ref()
    :spare(0)
    ,messageID(DATALINK_MESSAGE0)
    ,messageSize(sizeof(datalinkMessage0_ref))
    ,hcsum(0)
    ,csum(0)
  {};
  // HEADER
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare; /*  */
  int messageID; ///< \brief Message identifier.
  int messageSize;///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum; ///< \brief Modified Fletcher32 checksum of the complete message.
  // PAYLOAD
  char navStatus; ///< \brief status of nav system.
  char gpsStatus; ///< \brief status of gps.
  char sonarStatus; ///< \brief status of sonar.
  unsigned char overrun; ///< \brief frame overrun.
  char wow; ///< \brief weight on skids.
  char autopilot; ///< \brief autopilot engaged.
  char lavoid; ///< \brief limit avoidance.
  unsigned char motor; ///< \brief motor state.
  float time; ///< \brief onboard time.
  float pos[3]; ///< \brief Position of vehicle wrt. the datum
  float vel[3]; ///< \brief velocity of vehicle wrt. the datum.
  float q[4]; ///< \brief attitude wrt. the datum.
  float altitudeAGL; ///< \brief altitude above terrain.
} __attribute__((packed));

/** \brief Message struct for detailed status information of the A/C.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessage1_ref {
  datalinkMessage1_ref()
    :spare(0)
    ,messageID(DATALINK_MESSAGE1)
    ,messageSize(sizeof(datalinkMessage1_ref))
    ,hcsum(0)
    ,csum(0)
  {};
  // HEADER
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare; /*  */
  int messageID;///< \brief Message identifier.
  int messageSize; ///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum; ///< \brief Modified Fletcher32 checksum of the complete message.
  // PAYLOAD
  float time; ///< \brief onboard time.
  float gpsTime; ///< \brief onboard gps time.
  char numberOfSats; ///< \brief number of sats for GPS.
  char datarecordStatus; ///< \brief status of data recording.
  char safemode; ///< \brief safe mode status.
  char type; ///< \brief vehicle type.
  float delm[3]; ///< \brief actuators.
  float delf[1]; ///< \brief actuators.
  float delt[1]; ///< \brief actuators.
  int battery; ///< \brief battery voltage.
  int rpmYRD; ///< \brief rpm from yrd.
  int rpm; ///< \brief rpm from hub.
  char tx; ///< \brief transmitter status.
  char fuel; ///< \brief fuel status.
  char ycsStatus; ///< \brief YCS status.
  char yasStatus; ///< \brief YAS status.
  char yrdStatus; ///< \brief YRD status.
  char hubStatus; ///< \brief Hub status.
  char radarStatus; ///< \brief status of radar.
  char magnetStatus; ///< \brief status of magnetometer.
  float traj_x[3]; ///< \brief trajectory.
  float traj_v[3]; ///< \brief trajectory.
  float traj_a[3]; ///< \brief trajectory.
  float traj_q[4]; ///< \brief trajectory.
  float traj_psi; ///< \brief trajectory.
  float traj_vscale; ///< \brief trajectory.
  char traj_manIndex; ///< \brief trajectory.
  char traj_status; ///< \brief Status of the current trajectory execution. \sa GUST::TrajectoryStatus
  char visionStatus; ///< \brief vision satus.
  char missionStatus; ///< \brief visionNav status.
  char visionFormationStatus; ///< \brief visionFormation Status.
  unsigned char cameraControlStatus; ///< \brief cameraControl Status.
  char historyStatus; ///< \brief onboard computers history status.
  unsigned char batteryStatus; ///< \brief .
  char uplinkStatus[2]; ///< \brief uplink status.
  char lostComm; ///< \brief lost comm logic triggered.
  unsigned char hokuyoLaserStatus; ///< \brief Hokuyo laser status .
  unsigned char ubloxSNR; ///< \brief average.
  unsigned char ubloxHacc; ///< \brief (dm) horizontal accuracy.
  unsigned char ubloxSacc; ///< \brief (dm/s) speed accuracy.
  unsigned char ubloxPDOP; ///< \brief (*10).
  float pan; ///< \brief (deg).
  float tilt; ///< \brief (deg).
  float roll; ///< \brief (deg).
  float fovy; ///< \brief (deg).
  float G; ///< \brief load factor.
  float wind[3]; ///< \brief (ft/sec).
  float pointPos[3]; ///< \brief (ft).
  float datumLat; ///< \brief datum latitude (deg-N).
  float datumLon; ///< \brief datum longitude (deg-E).
  float datumAlt; ///< \brief datum altitude (ft).
} __attribute__((packed));

/** \brief Message struct for manipulating trajectories.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessageTrajectory_ref {
  datalinkMessageTrajectory_ref(TrajectoryCommand trajCommand=TRAJECTORY_STOP)
    :spare(0)
    ,messageID(DATALINK_MESSAGE_TRAJECTORY)
    ,messageSize(sizeof(datalinkMessageTrajectory_ref))
    ,hcsum(0)
    ,csum(0)
    ,command(trajCommand)
    ,waypoint(-1) //FIXME: Magic Numbers.
  {};
  // Header
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare;      ///< \brief Spare (sync) byte.
  int messageID;    ///< \brief Message identifier.
  int messageSize;  ///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum;       ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum;        ///< \brief Modified Fletcher32 checksum of the complete message.
  // Payload
  int command; ///< \sa GUST::TrajectoryCommand
  int waypoint;
} __attribute__((packed));

constexpr int maxNumberOfWaypoints = 100;

/** \brief Message struct for sending flight plans.
 * The datalinkMessageFlightPlan is a somewhat variable size message, with the
 * variety stemming from the number of utilized waypoint, stored in
 * waypoint[0] till waypoint[lastIndex].
 * GUST does not send the non-utilized waypoints and check functions at the 
 * receiving end actually check for that by comparing the actually received
 * byte count with the to-be-expected byte count indicated by messageSize.
 * As such, the messageSize is computed as
\code
messageSize = 
  sizeof(datalinkHeader_ref)
  + 4*sizeof(char) 
  + (lastIndex+1)*sizeof(datalinkMessageFlightPlanWaypoint_ref)
\endcode
 * and upper bounded at sizeof(datalinkMessageFlightPlan_ref), i.e. at a full
 * maxNumberOfWaypoints waypoints.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessageFlightPlan_ref {
  datalinkMessageFlightPlan_ref()
    :spare(0)
    ,messageID(DATALINK_MESSAGE_FLIGHTPLAN)
    ,messageSize(sizeof(datalinkMessageFlightPlan_ref))
    ,hcsum(0)
    ,csum(0)
    ,manIndex(0)
    ,lastIndex(0)
    ,extra(0)
    ,lostComm(0)
    ,waypoint{{0}}
  {};
 // HEADER
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare; /*  */
  int messageID; ///< \brief Message identifier.
  int messageSize; ///< \brief Total byte size of the \e utilized part of this message (incl. header).
  unsigned int hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum; ///< \brief Modified Fletcher32 checksum of the complete message.
  // PAYLOAD
  char manIndex; ///< \brief  <0 leaves it alone
  char lastIndex; /*  */
  char extra; ///< \brief  legacy
  char lostComm; ///< \brief  1 = this is the lost comm plan, 0 = otherwise
  /** \brief The collection of waypoints in this flight plan
   * Each FlightPlan message can hold up to maxNumberOfWaypoints individual 
   * waypoints. Each waypoint consumes a total of 20 bytes 
   * (sizeof(datalinkMessageFlightPlanWaypoint_ref))of space, hence \e waypoint
   * is a 2 dimensional array of maxNumberOfWaypointsx20 bytes.
   * \sa datalinkMessageFlightPlanWaypoint_ref
   * \todo Avoid those magic numbers.
   */
  char waypoint[maxNumberOfWaypoints][20] ; //FIXME: Magic Numbers,
} __attribute__((packed));

/** \brief Message struct for sending joystick commands.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessageUp0_ref {
  datalinkMessageUp0_ref()
    :spare(0)
    ,messageID(DATALINK_MESSAGE_UP0)
    ,messageSize(sizeof(datalinkMessageUp0_ref))
    ,hcsum(0)
    ,csum(0)
    ,k(0)
    ,time(0)
    ,throttleLever(0)
    ,rollStick(0)
    ,pitchStick(0)
    ,rudderPedal(0)
    ,button{0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}
  {  
  };
  // HEADER 
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare; /*  */
  int messageID; ///< \brief Message identifier.
  int messageSize; ///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum; ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum; ///< \brief Modified Fletcher32 checksum of the complete message.
  // PAYLOAD
  int k; ///< \brief discrete time onboard.
  float time; ///< \brief onboard time.
  float throttleLever; /*  */
  float rollStick; /*  */
  float pitchStick; /*  */
  float rudderPedal; /*  */
  unsigned char button[16]; ///< \brief Button Flags 
} __attribute__((packed));  


/** \brief Message struct to send remote commands.
 * \note As of SVN rev 11638 the spare byte needs to change inbetween 
 * messages as it is used to identify duplicate sendings of the same
 * message. */ 
struct datalinkMessageRmtCmd_ref {
  datalinkMessageRmtCmd_ref(std::string command="")
    :spare(0)
    ,messageID(DATALINK_MESSAGE_RMTCMD)
    ,messageSize(sizeof(datalinkMessageRmtCmd_ref))
    ,hcsum(0)
    ,csum(0)
    ,uniqueID(rand()%USHRT_MAX)
    ,empty(0x0)
    ,empty1(0x0)
    ,size(command.size()+1) // +1 to include a null-terminator
    ,data{0}
  {
    sprintf(reinterpret_cast<char*>(&data[0]),"%s",command.c_str());
  };
  static const int RMTCMD_MAXSIZE = 100;
  // Header
  unsigned char sync1 = SYNC_BYTE_1; ///< \brief First sync byte (0xA3).
  unsigned char sync2 = SYNC_BYTE_2; ///< \brief Second sync byte (0xB2).
  unsigned char sync3 = SYNC_BYTE_3; ///< \brief Third sync byte (0xC1).
  unsigned char spare;      ///< \brief Spare (sync) byte.
  int messageID;    ///< \brief Message identifier.
  int messageSize;  ///< \brief Total byte size of this message, incl. header.
  unsigned int hcsum;       ///< \brief Modified Fletcher32 checksum of the header only.
  unsigned int csum;        ///< \brief Modified Fletcher32 checksum of the complete message.
  
  //Payload
  /** \brief A random but unique(-ish) ID indentifying this remote command.
   * GUST compares the uniqueID of each received remote command message with the
   * uniquID of the last one it executed. If they differ, the newly received one
   * is executed as well and its uniquID gets stored for comparison with the 
   * next one received.
   */
  unsigned short uniqueID; 
  char empty;
  char empty1;
  /** \brief The actual size of the command.
   * I.e. the index of the null-terminator marking the end of the command char
   * array.
   */
  int size; 
  /** \brief The remote command to execute.
   * A null-terminated array of unsigned chars representing the ASCII chars making up 
   * the command.
   * \note The remote command message is always send in full, i.e. including the
   *   invalid entries in data after data[size]. As such, messageSize is fixed
   *   and independent of size, wich varies. 
   */
  unsigned char data[RMTCMD_MAXSIZE];
}__attribute__((packed));


struct datalinkMessageFlightPlanWaypoint_ref {
  char type; ///< \brief The ManeuverType of this waypoint
  char hdgAltMode; ///< \brief An overlay of the HeadingMode and the AltitueMode
  unsigned char vnom; ///< \brief in [ft/s]
  unsigned char anom; ///< \brief in [ft/s^2]
  float x[3]; ///< \brief The NED coordinates of the waypoint's position wrt. the esim datum in [ft]
  float psi; ///< \brief The heading at the waypoint in [degrees]
}__attribute__((packed));

struct maneuver_ref {
  maneuver_ref()
    :type(MAN_CUTCORNER)
    ,hdgMode(HDG_CONST)
    ,altMode(ALT_ABS)
    ,align1(0)
    ,align2(0)
    ,x{0}
    ,psi(0)
    ,vnom(0)
    ,anom(0)
    ,vt{0}
    ,vtmag(0)
    ,w{0}
  {};
  char type; ///< \brief 0=cut,1=thrgh,2=stop,3=eturn,4=land,5=pir,6=ext,7=replay,8=trk,9=chase,10=form,21=climb
  char hdgMode; ///< \brief 0=hdg specified, 1=beta, 2=point at point, 3=stick control
  char altMode; ///< \brief 0=datum, 1=terrain
  char align1; /*  */
  int align2; /*  */
  double x[3]; ///< \brief end position (ft)
  double psi; ///< \brief heading (deg)
  double vnom; ///< \brief velocity (ft/s)
  double anom; ///< \brief acceleration (ft/s2)
  double vt[3]; ///< \brief final velocity (ft/s)
  double vtmag; ///< \brief norm of final velocity (ft/s)
  double w[3]; ///< \brief unit vector along desired path
}__attribute__((packed));

struct flightplan_ref {
  flightplan_ref()
    :lastIndex(0)
    ,man{NULL}
  {};
  int lastIndex; ///< \brief The index of the last valid maneuver in the man array
  maneuver_ref* man[maxNumberOfWaypoints]; ///< \brief The list of maneuvers for this flightplan
}__attribute__((packed));


/** \brief Button Mapping
   *
   * From \e gcs.c :
   * - button[0]  = motionControls.safeOnButton;
   * - button[2]  = motionControls.safeOffButton;
   * - button[4]  = motionControls.wowUpButton;
   * - button[5]  = motionControls.shift;
   * - button[6]  = motionControls.wowDownButton;
   * - button[7]  = motionControls.crctrl;
   * - button[8]  = motionControls.peekLeftButton;
   * - button[9]  = motionControls.peekRightButton;
   * - button[10] = motionControls.goFastButton;
   * - button[11] = motionControls.gpsDeniedButton2;
   * - button[12] = motionControls.gpsDeniedButton1;
   * - button[15] = motionControls.manualOverride;
   */
enum ButtonMap {
  SafeOnButton          =  0 ,
  SafeOffButton         =  2 ,
  WowUpButton           =  4 ,
  ShiftButton           =  5 ,
  WowDownButton         =  6 ,
  CrCtrlButton          =  7 ,
  PeekLeftButton        =  8 ,
  PeekRightButton       =  9 ,
  GoFastButton          = 10 , ///< Activate the \e dash mode
  GpsDeniedButton2      = 11 ,
  GpsDeniedButton1      = 12 ,
  ManualOverrideButton  = 15 
  
};
  
} // end namespace GUST



#endif // DATALINKMESSAGES_H
