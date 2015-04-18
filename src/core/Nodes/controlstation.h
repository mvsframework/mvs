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

#ifndef CONTROLSTATION_H
#define CONTROLSTATION_H


#include "node.h"

class ControlStationPrivate; // forward declaration

class ControlStation : public Node{
  Q_OBJECT
  Q_PROPERTY(NodeId primaryNode READ primaryNode WRITE setPrimaryNode NOTIFY primaryNodeChanged) 

  template<typename Message, typename ParentNodeType, TopicOptions options>
  friend class Topic; // forward declaration 
  
public:
  /** \brief The XSD complex type containing settings elements for a ControlStation. */
  using Settings = XML::SAI::ControlStationNode_t;
  
Q_SIGNALS:
  void primaryNodeChanged(NodeId /*id*/);
  
public Q_SLOTS:
  /** \brief Set the Node this ControlStation should primarily control.
   * 
   * \param[in] id The NodeId of the node to control.
   * \return TRUE if the command was successfull; FALSE otherwise
   */
  bool setPrimaryNode(NodeId const & id);
  
  /** \brief Unset the primarily controlled Node. */
  void unsetPrimaryNode();
  
  /** \brief Send a command to a Node.
   * \param[in] command The command to send.
   * \param[in] id The NodeId of the Node to be commanded.
   */
  void sendCommand(IDL::Commands::CommandTypes const & command, NodeId const & id);
  
    
  /** \brief Command a remote Node to exectue a specific flightplan.
   * 
   * \param[in] plan The Flightplan to be send and executed but the remote Node.
   * \param[in] id The NodeId of the Node to be commanded.
   */
  void sendExternalPlanCommand(IDL::Data::Flightplan const & plan, NodeId const & id);

#ifdef WITH_GRAPHICS  
  virtual void buildBlipContextMenu(QMenu* parentMenu, NodeId const& id);
  virtual void buildFormationContextMenu(QMenu* parentMenu );
#endif // WITH_GRAPHICS
  
public:
  explicit ControlStation( Settings* const settings
                       /*, Node* parent */);
  virtual ~ControlStation();

  /** \brief Get the Type of this Node. */
  virtual Type type() const;
    
  /** \brief Get the NodeId of the primarily controlled Node.
   *
   * If no Node is currently controlled, the function returns the NodeId of the
   * ControlStation, i.e. this->id() .
   * 
   * \return The NodeId of the current primary Node. 
   */
  NodeId primaryNode() const;

#ifdef WITH_GRAPHICS  
  
  // @{
  /** \brief Set the marker for a currently dragged Blip.
   * \details Dragged Blips are treated as excludes when the R/F connected area
   * is updated.
   */
  void setDraggedBlip(NodeId const & id);
  /** \brief Unset the marker for dragged Blips. */
  void unsetDraggedBlip();
  // @}
  
  
  virtual void buildElementContextMenu(QMenu* parentMenu, const ElementId& id);
  
#endif // WITH_GRAPHICS
  
  
  /** \brief Set the target element via its ElementId.
   * \details Sets the target ID stored in the ownshipBlip. This is the entry 
   * point for processing after a target selection through a GCS operator. */
  virtual void setTargetId(ElementId const& id);
  virtual void unsetTarget();
  
  /** \brief Assign Nodes to formation positions. */
  virtual void realizeFormation();
  
protected:
  explicit ControlStation ( ControlStationPrivate& dd /*, Node* parent = nullptr*/);
  
  virtual void initializeItem();
  virtual void startItem();
  virtual void stopItem();
  virtual void unInitializeItem();
 
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::JoystickCommand const & message);
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::ModeChangeCommand const & message);
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::ExternalPlanCommand const & message);
  virtual Q_INVOKABLE void processReceivedMessage(NodeId const & sender, IDL::Messages::DualHopGuidance const & message);

private:
  Q_DISABLE_COPY(ControlStation)
  Q_DECLARE_PRIVATE(ControlStation);

};

#endif // CONTROLSTATION_H
