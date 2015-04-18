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

#ifndef NODE_H
#define NODE_H

#include "config.h"
#include "simitem.h"
#include "nodeid.h"
#include "blip.h"
#include "Environment/obstacle.h"
#include "XSD/sai.hxx"
#include "IDL/messages.h"
#include "IDL/messagesSupport.h"
#include "DDS/topicoptions.h"
#include "Geometry/geometry.h"
#include "Graph/comgraph.h"
#include "Graph/routegraph.h"

#ifdef WITH_GRAPHICS
  #include <QtWidgets/QGraphicsScene>
  #include <QtCore/QAbstractItemModel>
#endif // WITH_GRAPHICS


class NodePrivate; // forward declaration


/** \brief The basic Node (base) class.
 * 
 * The Node class provides the fundamental facilities all nodes in the network
 * need to have to partake in it, leaving any derived class to focus on 
 * implementing behavior.
 * 
 * As such, a Node can't have a parent; a feature which also enables Nodes to be
 * moved into different threads.
 * 
 * \sa https://qt-project.org/wiki/Threads_Events_QObjects
 * 
 * \todo Make a "Hop-Distance Map":
 * 1) For the origin point, compute the visbility poly. Color that with Color1.
 * 2) Discretize(*) the perimeter of the visibility polygon of Color1. Compute 
 * the visibiliyt polygons for all those points and merge the parts of them that 
 * are outside of Color1. Color that with Color2.
 * 3) continue...
 * (*): When discretizing, note that the perimeter of a visibility polygon is 
 * made up of three distinc types: sections that conicide with an obstacle 
 * (which are not interesting for the next hop computation), circular sections
 * at RANGE_MAX distance (which are interesting for hop range calculations), and 
 * radial spokes. The later are tricky: IF the spokes are attached to any of the
 * circular sections, the spokes are not interesting as the intersection of of 
 * spoke and circular section provides a coverage that cannot be extended from 
 * anywhere on the spoke. However, IF the spoke is delimited by obstacles on 
 * the outer side (the are always delimited by an obstacle perimeter section on the
 * inner side), then the spoke needs to be discretized as well as this indicates
 * a hop propagation that has a "kink" in it. (As opposed to a propagation on 
 * the circular perimeter, which could result in a line straight through that 
 * hop).
 * 
 * 
 */
class Node : public SimItem
{
  Q_OBJECT
  Q_PROPERTY(NodeId id READ id CONSTANT)
  Q_PROPERTY(ElementId targetId READ targetId WRITE setTargetId NOTIFY targetChanged)
  Q_PROPERTY(Type type READ type CONSTANT)
  
#if defined WITH_GRAPHICS
  Q_PROPERTY(bool routeGraphVisibility     READ isRouteGraphVisible      WRITE setRouteGraphVisibility     NOTIFY routeGraphVisibilityChanged     )  
  Q_PROPERTY(bool obstacleGraphVisibility  READ isObstacleGraphVisible   WRITE setObstacleGraphVisibility  NOTIFY obstacleGraphVisibilityChanged  )  
  Q_PROPERTY(bool comGraphVisibility       READ isComGraphVisible        WRITE setComGraphVisibility       NOTIFY comGraphVisibilityChanged       )  
  Q_PROPERTY(bool formationGraphVisibility READ isFormationGraphVisible  WRITE setFormationGraphVisibility NOTIFY formationGraphVisibilityChanged )  
  Q_PROPERTY(bool targetEnvironmentVisibility  READ isTargetEnvironmentVisible  WRITE setTargetEnvironmentVisibility   NOTIFY targetEnvironmentVisibilityChanged  )  
  Q_PROPERTY(bool comCorridorVisibility  READ isComCorridorVisible  WRITE setComCorridorVisibility   NOTIFY comCorridorVisibilityChanged  )  
#if defined DEBUG_GRAPHICS  
  Q_PROPERTY(bool bifurcationGraphVisibility      READ isBifurcationGraphVisible        WRITE setBifurcationGraphVisibility       NOTIFY bifurcationGraphVisibilityChanged      )    Q_PROPERTY(bool obstacleCellVisibility  READ areObstacleCellsVisible  WRITE setObstacleCellVisibility   NOTIFY obstacleCellVisibilityChanged  )  
  Q_PROPERTY(bool vroniGraphVisibility  READ isVroniGraphVisible  WRITE setVroniGraphVisibility   NOTIFY vroniGraphVisibilityChanged  )  
  Q_PROPERTY(bool faceGraphVisibility   READ isFaceGraphVisible   WRITE setFaceGraphVisibility    NOTIFY faceGraphVisibilityChanged   )  
  Q_PROPERTY(bool faceAssociationVisibility   READ isFaceAssociationVisible   WRITE setFaceAssociationVisibility    NOTIFY faceAssociationVisibilityChanged   )  
  Q_PROPERTY(bool bifurcationAssociationVisibility    READ isBifurcationAssociationVisible    WRITE setBifurcationAssociationVisibility     NOTIFY bifurcationAssociationVisibilityChanged    )  
  Q_PROPERTY(bool obstacleEdgeAssociationVisibility   READ isObstacleEdgeAssociationVisible   WRITE setObstacleEdgeAssociationVisibility    NOTIFY obstacleEdgeAssociationVisibilityChanged   )  
  Q_PROPERTY(bool obstacleVertexAssociationVisibility READ isObstacleVertexAssociationVisible WRITE setObstacleVertexAssociationVisibility  NOTIFY obstacleVertexAssociationVisibilityChanged )  
  Q_PROPERTY(bool rfGraphVisibility        READ isRfGraphVisible         WRITE setRfGraphVisibility        NOTIFY rfGraphVisibilityChanged        )  
  Q_PROPERTY(bool directedRfGraphVisibility        READ isDirectedRfGraphVisible         WRITE setDirectedRfGraphVisibility        NOTIFY directedRfGraphVisibilityChanged        )  
#endif // DEBUG_GRAPHICS
#endif // WITH_GRAPHICS
  
  Q_ENUMS(Type)
  
  template<typename Message, typename ParentNodeType, TopicOptions options>
  friend class Topic; // forward declaration 
   
public: 
  /** \brief The XSD complex type containing settings elements for a Node. */
  using Settings  = XML::SAI::GenericNode_t ;
  
  /** \brief The type identifier for a Node. */
  using Type      = IDL::Data::NodeType;
  
Q_SIGNALS:
  /** \brief Emitted whenever another Node has been detected.
   * \details This signal essentially announces that this node now has data for 
   * another node accessible via blip(id).
   */
  void blipAdded(NodeId /*id*/);
  
  /** \brief Emitted whenever the blip deletion process is started.
   * \details This signal essentially announces that it is no longer safe to 
   * access the blip with the given id. 
   */
  void aboutToDeleteBlip(NodeId /*id*/);
  
  
  /** \brief Emitted whenever the arena of the Node has been changed. */
  void arenaChanged();
  
  /** \brief Emitted whenever the COM graph of the Node has been changed. */
  void comGraphChanged();
  
  /** \brief Emitted whenever the target of the Node has changed. */
  void targetChanged(ElementId /*id*/);
  
  /** \brief Emitted when a COM formation connecting source and target is ready.*/
  void comFormationProposed(NodeId /*source*/, ElementId /*target*/);
  
#if defined WITH_GRAPHICS
  void routeGraphVisibilityChanged(bool /*isVisible*/);  
  void obstacleGraphVisibilityChanged(bool /*isVisible*/);  
  void comGraphVisibilityChanged(bool /*isVisible*/);  
  void formationGraphVisibilityChanged(bool /*isVisible*/);  
  void obstacleCellVisibilityChanged(bool /*isVisible*/);
  void targetEnvironmentVisibilityChanged(bool /*isVisible*/);
  void comCorridorVisibilityChanged(bool /*isVisible*/);
#if defined DEBUG_GRAPHICS
  void bifurcationGraphVisibilityChanged(bool /*isVisible*/);  
  void vroniGraphVisibilityChanged(bool /*isVisible*/);
  void faceGraphVisibilityChanged(bool /*isVisible*/);
  void faceAssociationVisibilityChanged(bool /*isVisible*/);
  void bifurcationAssociationVisibilityChanged(bool /*isVisible*/);
  void obstacleEdgeAssociationVisibilityChanged(bool /*isVisible*/);
  void obstacleVertexAssociationVisibilityChanged(bool /*isVisible*/);
  void rfGraphVisibilityChanged(bool /*isVisible*/);  
  void directedRfGraphVisibilityChanged(bool /*isVisible*/);  
#endif // DEBUG_GRAPHICS
#endif // WITH_GRAPHICS
       
public:

  /** \brief Constructor.
   * 
   * A Node (and all of its derived classes) is a standalone SimItem and as such
   * doesn't have a (SimItem) parent.
   * 
   * \param[in] settings The defining settings for this Vehicle.
   */
  explicit Node(Settings* const settings /*, SimItem* parent */);

  /** \brief Destructor. */
  virtual ~Node();
  
  /** \brief Get access to the XML settings used to initialize the Node. */
  virtual Settings const* settings() const;
    
  /** \brief Get the universally unique identifyer of this Node. */
  NodeId id() const;
  
  /** \brief Get the ElementId if the current target of this Node.
   * \details Gets the target ID stored in the ownshipBlip. */
  ElementId targetId() const;
  
  /** \brief Set the target element via its ElementId.
   * \details Sets the target ID stored in the ownshipBlip. This is the entry 
   * point for processing after a target selection through a GCS operator. */
  virtual void setTargetId(ElementId const& id);
  
  /** \brief Unset any set Target.
   * \details Sets the target ID stored in the ownshipBlip to NULL_ELEMENT_ID. */
  virtual void unsetTarget();
  
  /** \brief Get the Type of this Node. */
  virtual Type type() const;

  /** \brief Set the Object name if the Node has not yet initialized.
   * As some entities in the Datalink also    
   */
  void  setObjectName(QString const & name);

  /** \brief Add an obstacle.
   * \details Adds an obstacle to the Node's obstacle database.
   * \note Upon insertion of an obstacle, the arena becomes outdated. Use 
   * updateArena() to recompute the arena.
   * \sa isArenaCurrent()
   * \param[in] obstacle The obstacle to be added.
   */
  void addObstacle(Obstacle const & obstacle);
  
  /** \brief Remove an obstacle. 
   * \note This method is not yet implemented!
   * \todo This method is not yet implemented!
   */
  void removeObstacle(ElementId const& elementId);
  
  /** \brief Set the outer boundary for autonomous operations.
   * \details The outer boundary is the outer limit for any type of motion of
   * the Nodes. Upon (re-)setting the operational boundary, the arena and the 
   * route graph become invalid/outdated. Use updateArena() and 
   * updateRouteGraph() recompute the arena.
   * 
   * The outer boundary and the Obstacles (within that boundary)
   * form the Arena.
   * 
   * \sa updateArena()
   * \sa updateRouteGraph()
   * \param[in] ring The new shape of the operational boundary.
   */
  void setOperationalBoundary(Geometry::Polygon_2D::ring_type const & ring);
  
  /** \overload 
   * \deprecated {Use setOperationalBoundary(Geometry::Polygon_2D::ring_type const &).}
   */
  void setOperationalBoundary(Geometry::Polygon_2D const & polygon);
  
  /** \brief Flag whether all obstacles are incorporated into the arena.
   * \return TRUE if all obstacles are represented; FALSE otherwise.
   */
  bool isArenaCurrent() const;
  
  /** \brief Get a reference to the map of the environment. */
  Geometry::Polygon_2D& arena();
  
  /** \brief Get a copy of the map of the environment. 
   * \overload
   */
  Geometry::Polygon_2D  arena() const;
    
  /** \brief Recompute the arena based on the current obstacle data.
   * \details The arena is a complex polygon comprised of the operational 
   * boundary as its outer polygon and all the obstacles as (interior) holes.
   * \note When the arena changes, the route and derived graphs also change.
   */  
  void updateArena();
  
  /** \brief Get a copy of the COM graph. */
  ComGraph::Graph_t comGraph() const;
  
  /** \brief Get a copy of the route graph. */
  RouteGraph::Graph routeGraph() const;
  
#ifdef WITH_GRAPHICS

  //@{
  /** \brief Controls the visibility of the COM graph graphics. */
  bool isComGraphVisible() const;
  virtual void setComGraphVisibility(bool const & visibility);
  void showComGraph();
  void hideComGraph();
  //@}
  
  //@{
  /** \brief Controls the visibility of the route graph graphics. */
  bool isRouteGraphVisible() const;
  virtual void setRouteGraphVisibility(bool const & visibility);
  void showRouteGraph();
  void hideRouteGraph();
  //@}
  
  //@{
  /** \brief Controls the visibility of the obstacle graph graphics. */
  bool isObstacleGraphVisible() const;
  virtual void setObstacleGraphVisibility(bool const & visibility);
  void showObstacleGraph();
  void hideObstacleGraph();
  //@}
  
  //@{
  /** \brief Controls the visibility of the formation graph graphics. */
  bool isFormationGraphVisible() const;
  virtual void setFormationGraphVisibility(bool const & visibility);
  void showFormationGraph();
  void hideFormationGraph();
  //@}
  
  
  //@{
  /** \brief Controls the visibility of the obstacle cell graphics. */
  bool areObstacleCellsVisible() const;
  virtual void setObstacleCellVisibility(bool const & visibility);
  void showObstacleCells();
  void hideObstacleCells();
  //@}
  
  //@{
  /** \brief Controls the visibility of the target environment graphics. */
  bool isTargetEnvironmentVisible() const;
  virtual void setTargetEnvironmentVisibility(bool const & visibility);
  void showTargetEnvironment();
  void hideTargetEnvironment();
  //@}

  //@{
  /** \brief Controls the visibility of the COM corridor graphics. */
  bool isComCorridorVisible() const;
  virtual void setComCorridorVisibility(bool const & visibility);
  void showComCorridor();
  void hideComCorridor();
  //@}
  
#if defined DEBUG_GRAPHICS
  
  //@{
  /** \brief Controls the visibility of the bifurcation graph graphics. */
  bool isBifurcationGraphVisible() const;
  virtual void setBifurcationGraphVisibility(bool const & visibility);
  void showBifurcationGraph();
  void hideBifurcationGraph();
  //@}

  //@{
  /** \brief Controls the visibility of the VRONI graph graphics. */
  bool isVroniGraphVisible() const;
  virtual void setVroniGraphVisibility(bool const & visibility);
  void showVroniGraph();
  void hideVroniGraph();
  //@}

  //@{
  /** \brief Controls the visibility of the face graph graphics. */
  bool isFaceGraphVisible() const;
  virtual void setFaceGraphVisibility(bool const & visibility);
  void showFaceGraph();
  void hideFaceGraph();
  //@}

  //@{
  /** \brief Controls the visibility of the face-to-obstacle association graphics. */
  bool isFaceAssociationVisible() const;
  virtual void setFaceAssociationVisibility(bool const & visibility);
  void showFaceAssociation();
  void hideFaceAssociation();
  //@}

  //@{
  /** \brief Controls the visibility of the bifurcation-edge association graphics. */
  bool isBifurcationAssociationVisible() const;
  virtual void setBifurcationAssociationVisibility(bool const & visibility);
  void showBifurcationAssociation();
  void hideBifurcationAssociation();
  //@}

  //@{
  /** \brief Controls the visibility of the Obstacle to bifurcation-edge association graphics. */
  bool isObstacleEdgeAssociationVisible() const;
  virtual void setObstacleEdgeAssociationVisibility(bool const & visibility);
  void showObstacleEdgeAssociation();
  void hideObstacleEdgeAssociation();
  //@}

  //@{
  /** \brief Controls the visibility of the Obstacle to RouteGraph vertex association graphics. */
  bool isObstacleVertexAssociationVisible() const;
  virtual void setObstacleVertexAssociationVisibility(bool const & visibility);
  void showObstacleVertexAssociation();
  void hideObstacleVertexAssociation();
  //@}
  
  //@{
  /** \brief Controls the visibility of the R/F graph graphics. */
  bool isRfGraphVisible() const;
  virtual void setRfGraphVisibility(bool const & visibility);
  void showRfGraph();
  void hideRfGraph();
  //@}
  
  //@{
  /** \brief Controls the visibility of the R/F graph graphics. */
  bool isDirectedRfGraphVisible() const;
  virtual void setDirectedRfGraphVisibility(bool const & visibility);
  void showDirectedRfGraph();
  void hideDirectedRfGraph();
  //@}
#endif // DEBUG_GRAPHICS
  
  
  
  virtual void buildBlipContextMenu(QMenu* parentMenu, NodeId const& id);
  virtual void buildElementContextMenu(QMenu* parentMenu, const ElementId& id);
  
  /** \brief The context menu for a COM Formation. */
  virtual void buildFormationContextMenu(QMenu* parentMenu);
  
#endif // WITH_GRAPHICS
  
  /** \brief Get the timer id of the metronome timer with the given period.
   * 
   * This function gets the timer id of the Node's QTimer which fires every
   * period_ms milliseconds.
   * 
   * If no timer with the specified period exists, a corresponding timer is 
   * created. If Node::isRunning()==true, the timer get's started immediately.
   * 
   * All metronome timers are started when the Node starts (Node::run()) and 
   * they are killed when the Node is halted (Node::halt()). 
   * 
   * \param[in] period_ms The period of the timer in [ms]
   * \return The timer ID of the respective QTimer
   */
  int metronome(unsigned int period_ms);
  
  /** \brief Checks whether a Blip with the given ID is known to this node.
   * \param[in] id The NodeId to check for existense.
   * \return TRUE, if the Blip is known; FALSE otherwise. */
  bool blipExists(NodeId const & id) const;
  
  /** \brief Get a constant reference to the data stored for another Node in the network.
   * 
   * 
   * Get a reference to the blip data stored for the node with the given id.
   * The function assumes that the indicated node id is known to the node and
   * fails if this should not be the case. As such, some safeguarding and/or
   * checking is necessary.
   *
   * Option 1:
\code
try
{
  blip(id).name();
}
catch(std::out_of_range const& e)
{
  QString message(qualifiedName());
  message.append(QString(" : Blip(%1) doesn't exists.").arg(id));
  qCritical(message.toStdString().c_str());
}
\endcode
   *
   * Option 2:
\code
if( blipExists(id) )
{
  blip(id).name();
}
else
{ // blid(id) doesn't exist
  QString message(qualifiedName());
  message.append(QString(" : Blip(%1) doesn't exists.").arg(id));
  qCritical(message.toStdString().c_str());
}
\endcode
   *
   * \pre blipExists(id)==true;
   * 
   * \param[in] id The NodeId of the Blip in question.
   * \return The stored data for that Blip.
   */
  Blip const & blip(NodeId const & id) const;
  
  /** \brief Get a const reference to the data stored for the ownship. */
  Blip const & ownship() const;
  
  /** \brief Update the R/F polygon for the indicated node.
   * 
   * \param[in] id The id of the blip for which to compute the R/F polygon.
   * \param[in] minDistance (optional) The minimum distance the Blip needs to
   *  have moved before a new R/F polygon is computed.
   * \return TRUE, if a new R/F polygon has been computed; FALSE otherwise.
   */
  bool updateRfPoly(NodeId const & id,
                    double const & minDistance = 2.0 ); 
  
  
  /** \brief Update the FOV polygon for the indicated node.
   * 
   * \param[in] id The id of the blip for which to compute the FOV polygon.
   * \param[in] minDistance (optional) The minimum distance the Blip needs to
   *  have moved before a new FOV polygon is computed.
   * \return TRUE, if a new FOV polygon has been computed; FALSE otherwise.
   */
  bool updateFovPoly(NodeId const & id,
                    double const & minDistance = 2.0 ); 
  
  
  /** \brief Checks whether an Obstacle with the given ID is known to this node.
   * \param[in] id The ElementId to check for existense.
   * \return TRUE, if the Obstacle is known; FALSE otherwise. */
  bool obstacleExists(ElementId const & id) const;
  
  /** \brief Get a const reference to the data stored for the Obstacle.
   *
   * \pre obstacleExists(id)==true;
   * 
   * \sa Node::blip() for applicable safe implementation details.
   * 
   * \param[in] id The ElemetId of the Obstacle in question.
   * \return The stored data for that Obstacle.
   */
  Obstacle const & obstacle(ElementId const & id) const;
  
 
#ifdef WITH_GRAPHICS
  
  /** \brief Get the scene this node uses to display its graphics in. */
  QGraphicsScene* scene() const;
  
  /** \brief Set the scene this node should use to display its graphics in. */
  void setScene(QGraphicsScene* const scene);
  
  /** \brief Get the model/view interface to the blip data. */
  QAbstractItemModel* blipModel();
  
#endif // WITH_GRAPHICS
  
protected:
  
  /** \brief Constructor optimized for d-pointer inheritance.
   * See http://qt-project.org/wiki/Dpointer for more explanations.
   */
  explicit Node(NodePrivate & dd /*, SimItem* parent*/);
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();
  
  /** \brief Get access to the Node's mutex . 
   * Use this mutex with a QMutexLocker to serialze access to the Node.
   * \note Use at your own risk.
\code
// the mutex is not locked here
{ //  a section of code the needs to be locked 
  QMutexLocker locker(mutex());
  
  // the code the needs to operate when the mutex is locked goes here
  // ...
  
  // the QMutexLocker automatically releases the mutex upon its deletion
}
// the mutex is not locked here
\endcode
   */
  QMutex* mutex();
  
  /** \brief Node-level callback function for timer events.
   * \sa metronome(unsigned int period_ms)
   */
  virtual void timerEvent(QTimerEvent* event);
  
  /** \brief Get a reference to the stored data for another node in the network.
   * \overload
   * Non-constant version allowing to alter blip data.
   */
  Blip& blip(NodeId const & id);

  /** \brief Get a reference to the data stored for the ownship. */
  Blip& ownship();
  
  /** \brief Get a reference to the stored data for an Obstacle in the Environment.
   * \overload 
   * Non-constant version allowing to alter Obstacle data.
   */
  Obstacle& obstacle(ElementId const & id);

#ifdef WITH_RFSIMULATION  
  /** \brief Check whether the message sent from senderId is receivable or not.
   * This function essentially encompasses the R/F simulation and is used by
   * the Topic::ReaderListener to check whether to invoke 
   * Node::processReceivedMessage or not.
   * 
   * There are two cases for which this function behaves differently:
   * 
   * * The message sent \e does \e not contain information which could affect 
   *   the COM graph:
   *   In this case the function simply returns the current connectedness of
   *   the sender, effectively simply checking whether the sender is currently 
   *   a connected Node.
   *
   * * The message sent \e does contain information which could affect 
   *   the COM graph: (currently that is only IDL::Messages::State )
   *   The function processes the message information and makes a decision 
   *   based on the new data, effectively reversing the order the involved
   *   functions are invoked in.
   * 
   * \param[in] senderId The sender of the message.
   * \param[in] message The message sent.
   * \return TRUE if the message is receivable; FALSE otherwise.
   */
  template<typename Message>
  bool isMessageReceivable(NodeId const & senderId, Message const & message)
  { 
    // avoid erroneous error messages during startup
    if(!isRunning())
    { return false; }
    
    // messages from self are always receivable
    if( senderId == ownship().id() )
    { return true; }
    
    bool messageIsReceivable = false;
    
    if( blipExists(senderId) )
    { messageIsReceivable =  blip(senderId).isConnectedToHost(); }
    else
    { /* this is the first time a message from this blip has been detected. */
      QString message(qualifiedName());
      message.append(QString(": WARNING: received a %1 message from the unknown Node %2.")
                      .arg(QString(Message::TypeSupport::get_type_name()))
                      .arg(senderId.toQString()));
      qWarning("%s",message.toStdString().c_str());
    }
    
    return messageIsReceivable;
  };
#endif // WITH_RFSIMULATION
  
public Q_SLOTS :
  
  //
  // State Transition Command Slots
  // ==============================
  // These slots cause the emission of events which could trigger a transition.
  //
  
  /** \brief A Command Slot, posting the IdlCommand Event IDL::Commands::N_JOIN_NETWORK. */
  virtual void joinSwarm();   
  
  /** \brief A Command Slot, posting the IdlCommand Event IDL::Commands::N_LEAVE_NETWORK. */
  virtual void leaveSwarm();  

  
protected :  
  
  /** \brief Process incomming mode change commands.
   * 
   * This funciton handles incomming self-contained commands for the Node level:
   * - IDL::Commands::CommandTypes::N_JOIN_NETWORK
   * - IDL::Commands::CommandTypes::N_LEAVE_NETWORK
   *  
   * Being self-contained, these commands do not require any additional 
   * supplementing data to be processed as their meaning is unambigeous and 
   * known a priory by all parties involved.
   * 
   * \param[in] sender The origin/sender of the message.
   * \param[in] message The 
   */
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::ModeChangeCommand const & message);

  
  
  Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::State const & message);
  Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::Status const & message);
  Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::Datum const & message);
  Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::Flightplan const & message);
  
  /** \brief Create and add a new blip from the given data.
   * 
   * If there already is a blip with the given id, that blip is returned.
   * 
   * \param[in] id The id of the node this blip is representing.
   * \param[in] type The type of node this blip is representing.
   * \param[in] name The/a name for this blip.
   * \param[in] propertyList A serialized list of key/value QString pairs
   *  of properties.
   * \return The newly created (or found old) blip for the given id.
   */
  Q_INVOKABLE Blip* addNewBlip( NodeId const & id, const Node::Type& type
                              , const QString& name
                              , QStringList propertyList = QStringList()
                              );
  
  /** \brief Remove a blip.
   * \param[in] id The id of the blip to remove.
   */
  Q_INVOKABLE void deleteBlip( NodeId const & id);
  
private:
  Q_DISABLE_COPY(Node)
  Q_DECLARE_PRIVATE(Node)
  //NOTE: Node doesn't need a d_ptr as it inherits it from SimItem
};

Q_DECLARE_METATYPE(Node::Type);
Q_DECLARE_METATYPE(NodeId);

#endif // NODE_H
