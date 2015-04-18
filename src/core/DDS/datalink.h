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

#ifndef DATALINK_H
#define DATALINK_H




#include <ndds/ndds_cpp.h>

#include "Nodes/node.h"


class DatalinkPrivate;

/** \brief The main class dealing with the Dynamic Distribution System datalink.
 * This class provides an interface to RTI's Connext DDS system.
 *
 * \todo There might be an issue with selecting the participant_id for multiple
 *   participants on the same host with the same domain_id. (Compare page 8-53pp
 *   of the RTI Useres Manual.)
 *
 * \todo Create all DDS Entities in the disabled state and enable them during
 *  the initialization. As there is no DDS disable, that matches our code as
 *  there is no real un-initialize other than deconstruction.
 *
 */
class Datalink : public SimItem {
    Q_OBJECT

Q_SIGNALS:

    void newParticipantDetected(NodeId id, Node::Type type, QString name);

public:

    using Settings = XML::SAI::Datalink_t;

    /** \brief Constructor.
     * \pre Q_ASSERT(parent != nullptr);
     * \param[in] parent The Node this datalink is associated with.
     */
    explicit Datalink(Settings* const settings, Node* parent);

    /** \brief Destructor. */
    virtual ~Datalink();

    /** \brief Get the Datalink's parent */
    Node* parent() const;

    DDSDomainParticipant* participant();
    DDSPublisher* publisher();
    DDSSubscriber* subscriber();

    /** \brief Get the Domain ID.
     * When you create a DomainParticipant, you must specify a domain ID, which
     * identifies the communication channel across the whole system.
     */
    DDS_DomainId_t domain_id() const; // RTI_INT32

    /** \brief Get the RTPS Host ID.
     * If the value of rtps_host_id is set to DDS_RTPS_AUTO_ID, the IPv4 address
     * of the host is used as the host ID. If the host does not have an IPv4
     * address, then you should set this value to uniquely distinguish the host
     * from other nodes in the system.
     */
    DDS_UnsignedLong rtps_host_id() const;  // RTI_UINT32

    /** \brief Get the RTPS Application ID.
     * If the value of rtps_app_id is set to DDS_RTPS_AUTO_ID, the process (or
     * task) ID is used. There can be at most 256 distinct participants in a
     * shared address space (process) with a unique rtps_app_id.
     */
    DDS_UnsignedLong rtps_app_id() const; // RTI_UINT32

    /**  \brief Get the RTPS Instance ID.
     * If the value of rtps_instance_id is set to DDS_RTPS_AUTO_ID, a counter is
     * assigned that is incremented per new participant. Thus, together with
     * rtps_app_id, there can be at most 2^64 distinct participants in a shared
     * address space with a unique RTPS Globally Unique Identifier (GUID).
     */
    DDS_UnsignedLong rtps_instance_id() const;  // RTI_UINT32

    /** \brief Get the RTPS participant ID.
     * \sa http://community.rti.com/rti-doc/500/ndds.5.0.0/doc/html/api_cpp/structDDS__WireProtocolQosPolicy.html#af8282d78bd332c9b82d0112fab8c63d7
     */
    DDS_UnsignedLong rtps_participant_id() const; // RTI_UINT32

    /** \brief Get the globally unique 128 bit ID for the participant.
     *
     * This is a globally unique ID  generated from the domain, host, app, and
     * instance ids.
     */
    DDS_GUID_t participant_guid() const;

    /** \brief Get the unique DDS handle identifying the participant.
     *
     * This is a convenience function for participant()->get_instance_handle().
     *
     * The participant's handle contains the same data as the corresponding
     * GUID, just in a slighly different format.
     *
     * \sa participant_guid()
     */
    DDS_InstanceHandle_t participant_handle() const;



protected:
    /** \brief Constructor optimized for d-pointer inheritance.
      * See http://qt-project.org/wiki/Dpointer for more explanations.
      * \pre Q_ASSERT(parent != nullptr);
      * \param[in] parent The Node this datalink is associated with.
      */
    explicit Datalink(DatalinkPrivate & dd,Node* parent);

    /** \brief Performs the work necessary to initialize a Datalink.
     * \note When reimplementing this function in a derived class, make sure to
     * call the base class version \e before you do the derived class' work.
     */
    virtual void initializeItem();

    /** \brief Performs the work necessary to start a Datalink.
     * \note When reimplementing this function in a derived class, make sure to
     * call the base class version \e before you do the derived class' work.
     */
    virtual void startItem();

    /** \brief Performs the work necessary to stop a Datalink.
     * \note When reimplementing this function in a derived class, make sure to
     * call the base class version \e after you do the derived class' work.
     */
    virtual void stopItem();

    /** \brief Performs the work necessary to un-initialize a Datalink.
     * \note When reimplementing this function in a derived class, make sure to
     * call the base class version \e after you do the derived class' work.
     */
    virtual void unInitializeItem();


private:
    Q_DISABLE_COPY(Datalink)
    Q_DECLARE_PRIVATE(Datalink)
    //NOTE: Datalink doesn't need a d_ptr as it inherits it from SimItem
};

class datalink_error : public std::runtime_error
{
public:
  explicit datalink_error(std::string const& s)
  :runtime_error(s){};
  datalink_error(std::string const& s, DDS_ReturnCode_t const& retCode)
  :runtime_error(s),returnCode(retCode){};
  DDS_ReturnCode_t returnCode;
};

//############################################################################//
//                                                                            //
//  Inline Functions                                                          //
//                                                                            //
//############################################################################//




#endif // DATALINK_H
