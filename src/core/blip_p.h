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

#ifndef BLIPPRIVATE_H
#define BLIPPRIVATE_H

#include <QtCore/qglobal.h>


#include <string>
#include <amg.hpp>

#include "config.h"
#include "nodeid.h"
#include "Geometry/geometry.h"
#include "Graph/comgraph.h"
#include "Environment/element.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QMenu>

  #include "Graphics/coordinatesystemitem.h"
  #include "Graphics/blipitem.h"
  #include "Graphics/geometryitem.h"
  #include "Graphics/palette.h"
// #include "GNC/flightplan.h"
  #include "MVC/treeitem.h"
  #include "MVC/poditem.h"
  #include "MVC/BlipModel/positionitem.h"
  #include "MVC/BlipModel/attitudeitem.h"
  #include "MVC/BlipModel/velocityitem.h"
  #include "MVC/BlipModel/blipdebugitem.h"
#endif // WITH_GRAPHICS

class Blip; // forward declaration


/**
 * \todo The notion of "primary" and "secondary" data within the blip
 * is a littel missleading as it allows for some abiguition. What I'd like to 
 * implement in the long run is the following:
 * 
 * 
 */
class BlipPrivate {
public:
  BlipPrivate ( Blip* q );
  virtual ~BlipPrivate();
  
  
  //
  // Primary Data
  // ============
  // independet data that is set from the outside, e.g. position3D.
  // There are public accessors and mutators for this data.
  //
  
  
  /** \brief The NodeId of the Node this blip represents. */
  NodeId nodeId;
    
  /** \brief The type of the node this blip represents. */
  IDL::Data::NodeType type;
  
  /** \brief A description of the mode the node is in.
   * 
   * This reflects the overall state description of the node's state machine.
   */
  IDL::Data::ModeDescriptor modeDescriptor;
  
  /** \brief A description of the mode the node is in.
   * 
   * This reflects the overall state description of the node's state machine as
   * reported via the statusTopic.
   */
  QSet<IDL::Data::StateDescriptorId> mode;
   
  /** \brief The absolute position of a node in 3D space. */
  AMG::Vector position3D;
  
  /** \brief The velocity of the node. */
  AMG::Vector velocity;
  
  /** \brief The attitude of the node. */
  AMG::EulerAngleTupel attitude;
  
  /** \brief The current flight plan for this Blip. */
  IDL::Data::Flightplan currentFlightplan;
  
  /** \brief A flag indicating the validity of the stored Flightplan.
   * This flag is set to false when a Blip looses COM, every receipt of a 
   * Flightplan sets it to true. */
  bool currentFlightplanIsValid = false;

  /** \brief The index of the current waypoint in currentFlightplan.*/
  unsigned int currentWaypointIndex = 0;
  
  
  /** \brief The vertex descriptor for this blip in the COM graph. */
  ComGraph::Vertex_Descriptor_t comGraphVertex;
  
  /** \brief The horizontal Field-of-View in degree. */
  double horizontalFov_deg = 0; //NOTE: 0 degrees => no camera
  
  /** \brief The (datum) altitude of this blip's route graph (in meter).
   * \note The Datum is a NED frame, i.e. altitudes < 0 are "up".
   */
  double routeGraphAltitude = -100; 
  
  /** \brief The ID of the element set as the target. */
  ElementId targetElementId = NULL_ELEMENT_ID;
  
  /** \brief The ID of the element currently beint the closest obstacle. */
  ElementId closestObstacleId = NULL_ELEMENT_ID;
  
  //
  // Secondary Data
  // ==============
  // data that can be solemnly computed from the primary data, i.e. no further 
  // Blip-external data is necessary. An example would be  position2D.
  // There are only public accessors (if at all) for this data as it is 
  // immutuable from the outside.
  //
  
  /** \brief The position in 2D space used for the algorithms.
   * This is the position that vroni and visilibity use.
   */
  Geometry::Point_2D position2D;
  
  /** \brief The heading of the blip in degree, computed from it's velocity. */
  double heading_deg;
  
  /** \brief The speed of the blip, computed from it's velocity. */
  double speed;
  
  //
  // Tertiary Data
  // =============
  // this is blip specific data that is simply stored in the Blip as that is a 
  // natural place for it. Tertiary data is data that depends on primary and/or
  // secondary data in combination with further external data. An example would 
  // be the rfPolygon, as that depends on a Blip's position2D an a Node's 
  // environment2D.
  // There are public accessors as well as mutators for tertiary data.
  //
    
  /** \brief The (COM Graph) connectedness of this blip
   * 
   * TRUE if the Blip has a working COM connection to the hosting node, i.e. the
   * Blip is a member of the connected component of the hosting Node; 
   * FALSE otherwise.
   * 
   * \note \todo This will be false for the ownshipBlip iff the ownship has no other
   * Node within its R/F polygon.
   * 
   * \todo So how does this play together with WITH_RFSIMULATIONS ??
   * 
   * \internal  Well, technically every blip that gets an entry has to be 
   * connected as otherwise we wouldn't get messages from that blip...
   * However, in SITL simulations, this could be different...
   * 
   * 
   */
  bool isConnectedToHost = false;
    
  /** \brief The (COM Graph) connectedness of this blip to another Node of interest.
   * 
   * TRUE if the Blip has a working COM connection to the other Node of 
   * interest, i.e. the Blip is a member of the connected component of the
   * commanding ControlStation Node when the host is \e not part of the COM 
   * graph; FALSE otherwise.
   * 
   * \note This will be always false for the ownshipBlip.
   * \fixme This is technically only needed for hosts of the Vehicle type. 
   * 
   * \todo So how does this play together with WITH_RFSIMULATIONS ??
   * 
   * \internal  Well, technically every blip that gets an entry has to be 
   * connected as otherwise we wouldn't get messages from that blip...
   * However, in SITL simulations, this could be different...
   * 
   * 
   */
  bool isRfRelay = false;

  /** \brief The R/F coverage area of the Blip.
   * This polygon is the range-limited visibility polygon for a node at
   * rfPolygonCenter in the current environment. */
  Geometry::Polygon_2D  rfPolygon;   
  
  /** \brief The position for which the current rfPolygon is computed.
   * As nodes move, the postion2D might be updated more frequently than the 
   * rfPolygon (as this is a computationally heavy operation). This entry is 
   * keeps track of the center of the currently held rfPolygon, i.e. the 
   * position2D for which it has been computed. */ 
  Geometry::Point_2D    rfPolygonCenter;
  
  /** \brief The R/F coverage area at the last Waypoint of the Blip's Flightplan
   * This polygon is the range-limited visibility polygon for a node at
   * the last Waypoint of the current Flightplan in the current environment. */
  Geometry::Polygon_2D  lastWpRfPolygon;  
  
  /** \brief The visibiliyt polygon of a node. */
  Geometry::Polygon_2D  visPolygon;
  
  /** \brief The position for which the current visPolygon is computed.
   * As nodes move, the postion2D might be updated more frequently than the 
   * visPolygon (as this is a computationally heavy operation). This entry is 
   * keeps track of the center of the currently held visPolygon, i.e. the 
   * position2D for which it has been computed.
   */
  Geometry::Point_2D    visPolygonCenter;
     
  /** \brief The Field-of-View polygon of the Blip's camera. */
  Geometry::Polygon_2D  fovPolygon;
  
  /** \brief The position for which the current fovPolygon has been computed. */
  Geometry::Point_2D    fovPolygonCenter;
  
  /** \brief The heading angle for which the current fovPolygon has been computed. */
  double                fovPolygonHeading = 0;

  /** \brief The value to used to indicate a not-yet determined obstacle.
   * \sa NodePrivate::nan_obstacleIndex
   */
  static const int nan_obstacleIndex = -1; //FIXME: Magic Numbers
  
  /** \brief The index of the obstacle currently closest to the blip. */
  int closestObstacle_index = nan_obstacleIndex;
  

  
#ifdef WITH_GRAPHICS
  QColor mainColor = Qt::green;
  
  Blip::Palette const* palette = nullptr;
  
  /** \brief Apply the Palette to the current state/mode of the Blip. */
  void updatePalette();
  
  /** \brief The anchor item for all Blip graphics.
   * 
   * All Blip graphics items are children of the datumItem, which is the 
   * Qt graphics item that represents the Datum frame-of-reference of the Node
   * hosting this blip. This is necessary, as all numerical value are represented
   * wrt. to the hosting Node's Datum frame--which is represented by the 
   * datumItem.
   */
  QGraphicsItem* datumItem = nullptr; 
    
//   /** \brief Indicates whether to show/draw the rfPolygonItem. */
//   bool showRfPolygonItem = true;  
  
  Graphics_2D::CoordinateSystem* bodyCarriedFrameItem = nullptr;
  Graphics_2D::CoordinateSystem* bodyFrameItem = nullptr;
  
  Graphics_2D::BlipItem*  blipItem = nullptr;
  
  QGraphicsEllipseItem*       rfPolygonCenterItem = nullptr;
  Graphics_2D::Palette const* rfPolygonCenterPalette = nullptr;
  
  Graphics_2D::Polygon*       rfPolygonItem = nullptr;
  Graphics_2D::Palette const* rfPolygonPalette = nullptr;
  
  Graphics_2D::Polygon* lastWpRfPolygonItem = nullptr;
  
  /** \brief Graphics representing the last Waypoint of the current Flightplan. */
  QAbstractGraphicsShapeItem* lastWpItem = nullptr;

  /** \brief Update the (color) palette for the rfPolygonItem.
   * \details This function calls update() on the rfPolygonItem.
   */
  void updateRfPolygonPalette();

  /** \brief The graphics representing the blip's velocity. */
  Graphics_2D::Arrow*  velocityItem = nullptr;  

    
//   /** \brief Indicates whether to show/draw the fovPolygonItem. */
//   bool showFovItem = false;
  
  /** \brief The graphics representing the blip's FOV polygon. */
  Graphics_2D::Polygon* fovPolygonItem = nullptr;
  
  
  /** \brief The graphics representing the current Flightplan. */
  QGraphicsItemGroup* currentFlightplanItem = nullptr;

  
  
  /** \brief Update the currentFlightplanItem graphics.
   *  
   * For <tt>pastWpCount=1</tt> and <tt>futureWpCount=2</tt> the graphics would 
   * look like this:
   * ~~~~~
   * Waypoints : 0 ----- 1 ------ 2 ------ 3 ------- 4 ------ 5
   * Blip Pos  :             ^
   * Graphics  :         0        0========0=========0   
   * Current Wp:                  ^      
   * ~~~~~
   * 
   * The currentFlightplanGraphics are a fairly static thing and don't need to
   * be updated constantly, i.e. with every Blip movement. For the line 
   * connecting the Blip to the current waypoint, 
   * \see updateCurrentWaypointGraphics
   * 
   * \param[in] futureWpCount The number of future (next) waypoints, 
   * i.e. after the currentWaypoint, that should be drawn when plotting the 
   * flightplan. Counts less than 0 select all Waypoints.
   * \param[in] pastWpCount The number of past (passed) waypoints, 
   * i.e. before the currentWaypoint, that should be drawn when plotting the 
   * flightplan. Counts less than 0 select all Waypoints.
   */
  void updateCurrentFlightplanGraphics(const int& futureWpCount, const int& pastWpCount);
  
  /** \brief The number of future (next) waypoints drawn in flightplan graphcis. */
  int flightPlanGraphicsFutureWpCount = 2; //FIXME: Magic Numbers
    /** \brief The number of past (passed) waypoints drawn in flightplan graphcis.*/
  int flightPlanGraphicsPastWpCount = 0; //FIXME: Magic Numbers
  
  /** \brief Update the currentFlightplanItem graphics.
   * \overload
   * This function uses BlipPrivate::lightPlanGraphicsFutureWpCount and 
   * BlipPrivate::flightPlanGraphicsPastWpCount as parameters for 
   * updateCurrentFlightplanGraphics(const int& futureWpCount, const int& pastWpCount).
   */
  void updateCurrentFlightplanGraphics()
  {
    updateCurrentFlightplanGraphics(flightPlanGraphicsFutureWpCount,
                                    flightPlanGraphicsPastWpCount  );
  };
  
  
  
  /** \brief The graphics representing the path to the current waypoint. */
  QGraphicsItemGroup* currentWaypointItem = nullptr;

  /** \brief Update the currentWaypointItem graphics. 
   * This method updates the graphics connecting the Blip to the current 
   * waypoint.
   */
  void updateCurrentWaypointGraphics();
  
  /** \brief Generate the graphics representing a waypoint.
   * \note The caller assumes ownership of the memory, i.e. this function calls
   * <tt>new</tt> and does not delete that memory!
   * \param[in] position2D The coordinates where to place the graphics.
   * \param[in] actions TRUE if the waypoint has yet to be visited; FALSE if the
   * waypoint has already been passed.
   * \param[in] radius The radius of the graphical marker to use.
   * \return The graphics item.
   */
  QAbstractGraphicsShapeItem* generateWaypointGraphics( QPointF position2D, 
    bool active = false, qreal radius = 1);
  /** \overload */
  QAbstractGraphicsShapeItem* generateWaypointGraphics( const IDL::Data::Waypoint& waypoint,
    bool active = false, qreal radius = 1);
  
  
  //
  // model/view related entries 
  //
  
  /** \brief Respond to a request for data from a model/view-model. */
  QVariant modelData(int const & column , int role ) const;
  
  /** \brief Structure to hold the childrend of the BlipItem.
   * \note This cannot be constructed in BlipPrivate as the constructor needs
   * access to Blip!
   */
  struct ModelViewSubItems {
    ModelViewSubItems(BlipPrivate* const blip);
    ~ModelViewSubItems();
    PositionItem*   position  = nullptr;
    AttitudeItem*   orientation = nullptr;
    VelocityItem*   speed = nullptr;
    BlipDebugItem*  debug = nullptr;
  };
  ModelViewSubItems* mvSubItems = nullptr;
  
  QString position2dString; // QString("---.--° N, ---.--° E");
  
  QString generateModeDescription() const;
  
  
  
  
  
 
  /** \brief The menu of actions always available for a Blip. */
  QMenu persistentContextMenu;
 
  QAction* rfPolyVisibilityAction = nullptr;
  QAction* fovPolyVisibilityAction = nullptr;
  QAction* bodyFrameVisibilityAction = nullptr;
  
  /** \brief Update the persistent context menu actions to reflect the current state. */
  void updatePersistentContextMenu();
  
//   virtual void buildContextMenu(QMenu* parentMenu);
//   QList<QAction*> temporaryActions;
  
#endif // WITH_GRAPHICS

  
private:
  Q_DECLARE_PUBLIC(Blip);
  Blip* const q_ptr;
};

#endif // BLIPPRIVATE_H
