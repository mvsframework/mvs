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


#include "DDS/datalink.h"
#include "DDS/datalink_p.h"
#include "config.h"

#ifdef WITH_RTI_MONITORING
#include <ndds/monitor/monitor_common.h> // RTI Connext Monitoring library
#endif // WITH RTI_MONITORING


#include "Nodes/node.h"

#include <assert.h>
#include <QtCore/QDebug>

//############################################################################//
//                                                                            //
//  DatalinkPrivate                                                           //
//                                                                            //
//############################################################################//

DatalinkPrivate::DatalinkPrivate( Datalink* q, Datalink::Settings*const settings, Node* parent )
    :SimItemPrivate( q,settings )
    ,q_ptr( q )
    ,xmlSettings(settings)
{   //NOTE: Do _NOT_ use q_ptr or q in here, it _WILL_ break things!

}



void DatalinkPrivate::attachToBuiltInTopic( DDSDataReaderListener* listener,
        char const * topicName,
        DDS_StatusMask mask )
{
    auto builtin_subscriber = participant->get_builtin_subscriber();

    // builtin_reader for new publications: topic DDS_SUBSCRIPTION_TOPIC_NAME
    auto builtin_sub_reader = builtin_subscriber->lookup_datareader(topicName);
    if (builtin_sub_reader == nullptr) {
        qCritical("error getting builtin_sub_reader\n");
        return;
    }

    DDS_ReturnCode_t retcode = builtin_sub_reader->set_listener(listener, mask);
    if (retcode != DDS_RETCODE_OK) {
        qCritical("set_listener builtin_sub_reader error %d\n", retcode);
        return;
    }

    DDS_DataReaderQos defReaderQoS;
    builtin_sub_reader->get_qos(defReaderQoS);

    if( defReaderQoS.liveliness.lease_duration.is_infinite() )
    {
        qDebug() << "For topic" << topicName <<"the built-in reader liveliness lease duration is INFINITY.";
    }
    else
    {
        qDebug() << "For topic" << topicName <<"the built-in reader liveliness lease duration is"
                 << QString("%1.%2")
                 .arg(defReaderQoS.liveliness.lease_duration.sec)
                 .arg(defReaderQoS.liveliness.lease_duration.nanosec)
                 << "seconds.";
    }

}


DatalinkPrivate::~DatalinkPrivate()
{
    Q_Q(Datalink);
    Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": ~DatalinkPrivate()";
}


void DatalinkPrivate::initializeDatalink()
{
    Q_Q(Datalink);
    Q_ASSERT(q);

    DDS_ReturnCode_t            retcode;

    Node* parentNode = q->parent();

    {   /** create the participant...
         *
         * \internal
         * The code related to setting up the "spies" via the built-in subscriber is
         * based upon an example from the RTI knowledge base:
         * http://community.rti.com/kb/how-can-i-detect-new-datawriters-and-datareaders-joining-domain
         */

        {   /** \internal Factory */

            DDS_DomainParticipantFactoryQos   factory_qos;

            // To conserve memory, builtin Subscribers and DataReaders are created
            // only if and when you look them up. Therefore, to avoid missing
            // builtin data, you should look up the builtin readers before the
            // DomainParticipant is enabled.

            // Modify the Factory's QoS so that DomainParticipants are not automatically enabled when created
            DDSTheParticipantFactory->get_qos(factory_qos);


            factory_qos.entity_factory.autoenable_created_entities = DDS_BOOLEAN_FALSE;


            if(false) // DEBUG
            {   /** \internal Increasing the verbosity of DDS internal logging.
                 * copied from http://community.rti.com/comment/1979#comment-1979 */
                factory_qos.logging.category = NDDS_CONFIG_LOG_CATEGORY_ENTITIES ;
                factory_qos.logging.verbosity = NDDS_CONFIG_LOG_VERBOSITY_STATUS_LOCAL;
            }

            DDSTheParticipantFactory->set_qos(factory_qos);

        }





        // Modify participant_qos settings here, if necessary
        retcode = DDSTheParticipantFactory->get_default_participant_qos(participant_qos);
        if (retcode != DDS_RETCODE_OK) {
            qCritical("get default participant qos error %d\n", retcode);
            throw datalink_error("Get default participant QOS error.",retcode);
            return ;
        }

        // Increase type code serialized length from 256 (default) to 64K (maximum).
        // This ensures we can receive more involved type codes
        participant_qos.resource_limits.type_code_max_serialized_length = 0xffff; //FIXME: Magic Numbers

        // Increase receiver buffer size. The UDPv4 transport resource settings
        // will be increased after participant is created.
        participant_qos.receiver_pool.buffer_size = 65535; //FIXME: Magic Numbers

        {   /** transfer the name of the node to the DDS participant
             *
             *\note This is a one time thing, i.e. later updates of the name of the
             * parent node will (and actually can) \e not be propagated to the DDS
             * participant!
             *
             */
            DDS_String_replace(&participant_qos.participant_name.name,
                               parentNode->objectName().toStdString().c_str());
        }

        {   /// transfer the type of the node to a DDS participant property
            QHash<QString,QString> props;

            props["type"]=enumToQString(parentNode->type());
            qDebug() << q->qualifiedName() << ": Parent Node is of type" << props["type"];

            int hFov = 0;
            if( parentNode->settings()->camera().present() )
            {
                hFov = parentNode->settings()->camera().get().hFov();
            }
            props["hFov"]=QString("%1").arg(hFov);

#ifdef WITH_RTI_MONITORING
            {
                /** \internal Enabling monitoring functionality for this participant
                 * (using the RTI Monitor Library)
                 * NOTE: Compare RTI_Monitoring_Library_GettingStarted.pdf */
                props["rti.monitor.library"]="rtimonitoring";

                char buffer[17];
                sprintf(buffer,"%p",RTIDefaultMonitor_create);
                props["rti.monitor.create_function_ptr"] = buffer;
            }
#endif // WITH RTI_MONITORING      

            auto old_props = participant_qos.property.value;

            DDS::appendHashToProperty(props, participant_qos.property.value);

            auto new_props = participant_qos.property.value;

            //NOTE: don't free the memory pointed to by type.name and value, as
            // in that case the newly added property has a "\null" name and type. :(
            //
//       DDS_String_free(type.name); //TODO: who would do the clean up of this?
//       DDS_String_free(type.value); //TODO: who would do the clean up of this?
        }



        {   /// set discovery and liveliness parameters

            /** \internal
             * Some related articles on the RTI community forum / knowledge base:
             * - http://community.rti.com/kb/why-does-my-subscriber-report-multiple-liveliness-change-events-when-i-kill-my-publisher
             * - http://community.rti.com/forum-topic/how-can-i-react-participant-going-stale
             */


            // The time (in seconds) this node has had to been silent before other participants
            // declare it "stale" or "dead"
            //FIXME: I never ever got this to work... :/
            participant_qos.discovery_config.participant_liveliness_lease_duration.sec = 5; //NOTE: the default is 100

            // The time intervall (in seconds) in which the participant will assert
            // its liveliness (i.e. it will send out an liveliness asserting message)
            participant_qos.discovery_config.participant_liveliness_assert_period.sec  = 3; //NOTE: the default is 30

        }





        participant_listener = new NodePrivate::Participant_Listener(parentNode);
        DDS_StatusMask mask = NodePrivate::Participant_Listener::implementedCallbacks;


        // Create the DomainParticipant (which will start out disabled)
        participant  = DDSDomainParticipantFactory::get_instance()->
                       create_participant(
                           domainId              // Domain ID
                           ,participant_qos      // QoS
                           ,participant_listener // Listener
                           ,mask                 // Listener Mask
                       );
        Q_ASSERT(participant);

        if (participant == nullptr) {
            qCritical("create_participant error %d\n", retcode);
            throw datalink_error("Create participant error.",retcode);
            return;
        }
    }

    {   /** change the default UPDv4 socket layer settings.
         *
         * \internal
         * From the knowledge base entry related to the "spies":
         * "A a side note not related to builtin topics but related to the example
         * attached to this solution, we increase the typecode serialized length
         * from 256 (the default) to 64K (the maximum) to make sure we can receive
         * more involved typecodes. The typecode data size varies from data type to
         * data type (the more complex the structure, the larger the typecode). The
         * DomainParticipant's resource_limits.type_code_max_serialized_length
         * controls the maximum typecode size that can be sent and received by the
         * participant. RTI Connext’s default value is 256, which is adequate for
         * only very simple data structures (for instance, a simple struct with no
         * more than 8 primitive members)."
         */

        NDDS_Transport_UDPv4_Property_t property = NDDS_TRANSPORT_UDPV4_PROPERTY_DEFAULT;
        NDDS_Transport_Property_t& propref = reinterpret_cast<NDDS_Transport_Property_t&>(property); //FIXME: reinterpret_cast!

        // The default maximum data size is 8K. This is determined by the 9K maximum
        // socket buffer size and message_size_max, as defined in the UDPv4 builtin
        // transport.
        // To change the properties of the UDPv4 builtin transport, refer to the
        // online documentation (Programming How-To's ->Transport Use Cases).
        //      1. Turn off auto-enable of newly created participants (see above)
        //      2. Get the UDPv4 property
        //      3. Modify the property
        //      4. Set the property
        //      5. Enable the participant



        retcode = NDDSTransportSupport::get_builtin_transport_property(
                      participant,
                      DDS_TRANSPORTBUILTIN_UDPv4,
                      propref);
        if (retcode != DDS_RETCODE_OK)
        {
            qCritical() << q->qualifiedName()
                        << ": DDS error getting builtin transport property:" << retcode;
        }

        /* Increase the UDPv4 maximum message size to 64K (large messages) */
        property.parent.message_size_max = 65535; //FIXME: Magic Numbers
        property.recv_socket_buffer_size = 65535; //FIXME: Magic Numbers
        property.send_socket_buffer_size = 65535; //FIXME: Magic Numbers

        retcode = NDDSTransportSupport::set_builtin_transport_property(
                      participant
                      ,DDS_TRANSPORTBUILTIN_UDPv4
                      ,propref);
        if (retcode != DDS_RETCODE_OK)
        {
            qCritical() << q->qualifiedName()
                        << ": DDS error seting builtin transport property:" << retcode;
        }
    }

    {   /** \internal set the liveliness duration for the builtin reader for the topic
         *  DDS_PARTICIPANT_TOPIC_NAME */

        auto builtin_subscriber = participant->get_builtin_subscriber();
        Q_ASSERT( builtin_subscriber );


        auto builtin_sub_reader = builtin_subscriber->lookup_datareader(DDS_PARTICIPANT_TOPIC_NAME);
        Q_ASSERT( builtin_sub_reader );

        DDS_DataReaderQos defReaderQoS;
        builtin_sub_reader->get_qos(defReaderQoS);

        defReaderQoS.liveliness.kind = DDS_AUTOMATIC_LIVELINESS_QOS; ///NOTE: Default is DDS_AUTOMATIC_LIVELINESS_QOS
        defReaderQoS.liveliness.lease_duration = DDS_Duration_t::from_millis(1250); ///FIXME: Magic numbers: BuiltIn Reader Liveliness Duration 1.25s

        builtin_sub_reader->set_qos(defReaderQoS);
    }



    {   /// attach listeners to the built-in topics
        builtInTopic_Publications_Listener = new NodePrivate::BuiltInTopic_Publications_Listener(parentNode);
        attachToBuiltInTopic(builtInTopic_Publications_Listener, DDS_PUBLICATION_TOPIC_NAME
                             /*listener mask = DDS_DATA_AVAILABLE_STATUS */
                            );

        builtInTopic_Subscriptions_Listener = new NodePrivate::BuiltInTopic_Subscriptions_Listener(parentNode);
        attachToBuiltInTopic(builtInTopic_Subscriptions_Listener, DDS_SUBSCRIPTION_TOPIC_NAME
                             /*listener mask = DDS_DATA_AVAILABLE_STATUS */
                            );

        DDS_StatusMask participantListenerMask =
            NodePrivate::BuiltInTopic_Participant_Listener::implementedCallbacks;

        builtInTopic_Participant_Listener = new NodePrivate::BuiltInTopic_Participant_Listener(parentNode);
        attachToBuiltInTopic(builtInTopic_Participant_Listener, DDS_PARTICIPANT_TOPIC_NAME,
                             /*listener mask */ participantListenerMask
//       /*listener mask */ DDS_LIVELINESS_CHANGED_STATUS
                            );
    }

    {   /// create the publisher ...



        //NOTE: To customize publisher QoS, use the configuration file USER_QOS_PROFILES.xml
        publisher = participant->create_publisher( DDS_PUBLISHER_QOS_DEFAULT,
                    nullptr,  // listener
                    DDS_STATUS_MASK_NONE
                                                 );
        Q_ASSERT(publisher);
    }

    {   /// create the subscriber ...
        subscriberListener = new SubscriberListener;
//     DDS_StatusMask mask =   DDS_DATA_ON_READERS_STATUS
//                           | DDS_DATA_AVAILABLE_STATUS ;
        DDS_StatusMask mask = subscriberListener->implementedCallbacks;


        //NOTE: To customize the subscriber QoS, use the configuration file USER_QOS_PROFILES.xml
        subscriber = participant->create_subscriber( DDS_SUBSCRIBER_QOS_DEFAULT
                     , subscriberListener         // Listener
                     , mask // Listener Mask
                                                   );
        Q_ASSERT(subscriber);
    }


    // enable the factory entities in DDS.
    // NOTE: The participant is disabled as the auto-enable QoS of the factory has
    // ben set to false. However, that setting is still at it's default "true" for
    // the participant itself, meaning that the subscriber and publisher should
    // auto-enable once the participant is enabled. Hence it should be sufficient
    // to just enable the participant...

    retcode = participant->enable();
    if(retcode != DDS_RETCODE_OK)
    {
        qCritical("\"%s\" : Initialization failed, couldn't enable the participant. RETCODE = %s.",
                  q->qualifiedName().toStdString().c_str(),
                  DDS::enumToQString(retcode).toStdString().c_str() );
        throw datalink_error("Participant couldn't be enabled", retcode);
        return;
    }

    {   /** Configure the participant to ignore itself,
        * i.e. don't let dataReaders receive matching publications from
        * dataWriters of the same participant, i.e. the same Node.
        * This effectively means that a Node will only "receive" (well, process,
        * I guess) data from other Nodes
        * \note From the RTI API manual: "There is no way to reverse this
        * operation."
        */
        retcode = participant->ignore_participant(participant->get_instance_handle());

        if( retcode != DDS_RETCODE_OK )
        {
            qCritical("ignore_participant error %d\n", retcode);
            throw datalink_error("Ignore participant error.", retcode);
            return;
        }
    }

    {   // GUID Debug output
        qDebug() << q->qualifiedName() << "Ownship Instance Handle:" << participant->get_instance_handle();

        participant->get_qos(participant_qos);
        qDebug() << "RTPS Host ID        :"
                 << participant_qos.wire_protocol.rtps_host_id << ","
                 << QString::number(participant_qos.wire_protocol.rtps_host_id,16).toUpper().prepend("0x");
        qDebug() << "RTPS App ID         :"
                 << participant_qos.wire_protocol.rtps_app_id << ","
                 << QString::number(participant_qos.wire_protocol.rtps_app_id,16).toUpper().prepend("0x");
        qDebug() << "RTPS Instance ID    :"
                 << participant_qos.wire_protocol.rtps_instance_id << ","
                 << QString::number(participant_qos.wire_protocol.rtps_instance_id,16).toUpper().prepend("0x");

        /** \internal From RTI Api documentation ([local]/RTI/ndds.5.1.0/doc/html/api_cpp/classDDSDomainParticipantFactory.html#af7c7137eccd41ffa9351d577fb088417):
         * "If you want to create multiple participants on a given host in the same
         * domain, make sure each one has a different participant index (set in the
         * DDS_WireProtocolQosPolicy). This in turn will ensure each participant
         * uses a different port number (since the unicast port numbers are
         * calculated from the participant index and the domain ID)." */
        qDebug() << "RTPS Participant ID :"
                 << participant_qos.wire_protocol.participant_id << ","
                 << QString::number(participant_qos.wire_protocol.participant_id,16).toUpper().prepend("0x");
        qDebug() << "DDS Domain ID :"
                 << participant->get_domain_id() << ","
                 << QString::number(participant->get_domain_id(),16).toUpper().prepend("0x");
    }


    // get an update version of the participant QoS settings now that it has been
    // enabled.
    retcode = participant->get_qos(participant_qos);
    Q_ASSERT( retcode == DDS_RETCODE_OK );

}


void DatalinkPrivate::unInitializeDatalink()
{
    if(SimItem::Status::Constructed == currentStatus)
    {
        return;
    }

    Q_Q(Datalink);
    Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": DatalinkPrivate::unInitializeDatalink()";

    //
    // delete the DDS infrastructure
    //
    {
        DDS_ReturnCode_t retcode;

        {   // delete publisher and subscriber individually
            retcode = publisher->delete_contained_entities(); // delete all datawriters.
            if (retcode != DDS_RETCODE_OK)
            {}


            retcode = participant->delete_publisher(publisher);
            if (retcode != DDS_RETCODE_OK)
            {}
            publisher = nullptr;


            retcode = subscriber->delete_contained_entities(); // delete all datareaders.
            if (retcode != DDS_RETCODE_OK)
            {}


            retcode = participant->delete_subscriber(subscriber);
            if (retcode != DDS_RETCODE_OK)
            {}
            subscriber = nullptr;
        }

        retcode = participant->delete_contained_entities(); // delete all entities
        if (retcode != DDS_RETCODE_OK) {
            std::cerr << "! Unable to delete participant contained entities: " << retcode << std::endl;
            qCritical("\"%s\" : ! Unable to delete participant contained entities: %d",
                      q->qualifiedName().toStdString().c_str(),
                      static_cast<int>(retcode));
        }

        retcode = DDSDomainParticipantFactory::get_instance()->delete_participant(
                      participant);
        if (retcode != DDS_RETCODE_OK) {
            std::cerr << "! Unable to delete participant: " << retcode << std::endl;
            qCritical("\"%s\" : ! Unable to delete participant: %d",
                      q->qualifiedName().toStdString().c_str(),
                      static_cast<int>(retcode));
        }
        participant = nullptr;

        /** \internal From $(NDDSHOME)/example/CPP/helloWorld/HelloWorld_publisher.cxx :
         *
         * "RTI Connext provides finalize_instance() method on domain participant
         * factory for people who want to release memory used by the participant
         * factory singleton. Uncomment the following block of code for clean
         * destruction of the singleton."
         *
         * This cannot be called here, as the singleton can't be deleted as there
         * might be other Nodes who still need it.
         *
         */
//     retcode = DDSDomainParticipantFactory::finalize_instance();
//     if (retcode != DDS_RETCODE_OK)
//     {
//       std::cerr << "! finalize_instance error " << retcode << std::endl;
//       qCritical("\"%s\" : ! finalize_instance error : %d",
//             q->qualifiedName().toStdString().c_str(),
//             static_cast<int>(retcode));
//     }



        delete subscriberListener;
        subscriberListener = nullptr;

        delete builtInTopic_Publications_Listener;
        builtInTopic_Publications_Listener = nullptr;

        delete builtInTopic_Subscriptions_Listener;
        builtInTopic_Subscriptions_Listener = nullptr;

        delete builtInTopic_Participant_Listener;
        builtInTopic_Participant_Listener = nullptr;

        delete participant_listener;
        participant_listener = nullptr;

    }

}


//############################################################################//
//                                                                            //
//  Datalink                                                                  //
//                                                                            //
//############################################################################//

/** \note This constructor does NOT do any work!
 * All Datalink level construction needs to happen in
 * Datalink(DatalinkPrivate & dd, ...)!
 */
Datalink::Datalink( Settings*const settings
                    ,Node* parent )
    :Datalink(*new DatalinkPrivate(this,settings,parent),parent)
{}

Datalink::Datalink( DatalinkPrivate& dd, Node* parent )
    : SimItem(dd, parent) // Datalinks have Nodes as parents
{
    Q_ASSERT(parent != nullptr); // the Node parent must be valid.

    Q_D(Datalink);
    Q_ASSERT(d);



    /** \internal
     * This will only indicate the status of the Datalink, any derived classes
     * will not be constructed at this point!
     */
    d->setStatus(Status::Constructed);
}

Datalink::~Datalink()
{
//   qDebug() << objectName() << ": ~Datalink()";
    Q_D(Datalink);
    Q_ASSERT(d);
    d->unInitializeDatalink();
}

Node* Datalink::parent() const
{
    return dynamic_cast<Node*>(SimItem::parent());
}



void Datalink::initializeItem()
{
    SimItem::initializeItem();
    Q_D(Datalink);
    Q_ASSERT(d);
    d->initializeDatalink();
}


void Datalink::startItem()
{
    SimItem::startItem();
    /** \internal
     * As far as I understand, there is no real way of starting or suspending the
     * dataWriter(s). As such, the "correct" thing to do here would maybe be to
     * move the creating of the topics into this routine, i.e.
     * Datalink::startItem(). This would then announce the "creation" of a/the
     * writer(s) to the dds network. Consequently, this would also meant that
     * Datalink::stopItem() destroys the topics, resulting in the detaching of
     * the writer(s) from the DDS network...
     */
//   publisher()->resume_publications(); //TODO: does this really "start" sending any data?
//   subscriber()->begin_access(); //TODO: does this really "start" receiving any data?
}

void Datalink::stopItem()
{
    SimItem::stopItem();
    /** \internal See the comment in Datalink::startItem(); */
//   publisher()->suspend_publications();//TODO: does this really "stopt" sending data?
//   subscriber()->end_access(); //TODO: does this really "stop" receiving any data?
}

void Datalink::unInitializeItem()
{
//   qDebug() << objectName() << ": un-initializing.";
    Q_D(Datalink);
    Q_ASSERT(d);
    d->unInitializeDatalink();
    SimItem::unInitializeItem();
}


DDSDomainParticipant* Datalink::participant()
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    Q_ASSERT(d->participant );
    return d->participant;
}

DDSPublisher* Datalink::publisher()
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    Q_ASSERT( d->publisher );
    return d->publisher;
}

DDSSubscriber* Datalink::subscriber()
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    Q_ASSERT( d->subscriber );
    return d->subscriber;
}



DDS_DomainId_t Datalink::domain_id() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return d->participant->get_domain_id();
}

DDS_UnsignedLong Datalink::rtps_host_id() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return  d->participant_qos.wire_protocol.rtps_host_id;
}

DDS_UnsignedLong Datalink::rtps_app_id() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return  d->participant_qos.wire_protocol.rtps_app_id;
}

DDS_UnsignedLong Datalink::rtps_instance_id() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return  d->participant_qos.wire_protocol.rtps_instance_id;
}

DDS_UnsignedLong Datalink::rtps_participant_id() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return d->participant_qos.wire_protocol.participant_id;
}


/** \internal
 * From http://community.rti.com/kb/what-does-presparticipantassertremoteparticipantassert-due-different-ro-area-error-message-mean :
 * "Each DomainParticipant has a Globally Unique Identifier (GUID) that is used
 * to uniquely identify the application in its communications with other
 * applications. Each object within the participant also has its own GUID,
 * which consists of its application's GUID plus a unique identifier for that
 * particular object.
 *
 * The GUID is supposed to be unique; if two DomainParticipants have the same
 * GUID, this will cause communications problems, including the message above.
 * Generally, these problems will occur either if there are two applications
 * that are using the same GUID simultaneously, or if a new application starts
 * up with a GUID that was used by some previous (now dead) application, but
 * the dead application's liveliness duration (as defined by the LIVELINESS QoS
 * policy on the data readers and data writers) has not yet elapsed.
 *
 * You can use the GUID displayed in the message to determine which node, and
 * perhaps which application, is causing the GUID problem. The displayed
 * hexadecimal numbers [e.g. "c0c80aa3 7e520001"] consist of:
 *
 * - 32 bits for the host ID [4 bytes; 0xc0c80aa3]
 *   The host ID is currently based upon the IP address (although this might
 *   change in future versions). In the above case, a GUID of c0c80aa3
 *   translates to IP address 192.200.10.163. This will allow you to determine
 *   the node having the GUID problem.
 *
 * - 16 low bits for the process ID [2 bytes; 0x7e52]
 *   If the originating node is a Windows system, the relevant "process ID" will
 *   be the value of GetCurrentProcessId().
 *   On a VxWorks system, it will be the task ID of a specific task.
 *   On a Linux system, it will be the process ID that you can see as the
 *   output of the command ps -ef.
 *
 * - 8 bits for an internal counter. [1 byte; 0x00]
 *   This counter allows an application to create multiple DomainParticipants
 *   in the same domain.
 *
 * - 8 bits containing a constant value [1 byte; 0x01].
 *
 */
DDS_GUID_t Datalink::participant_guid() const
{
    return DDS::convertToGuid(participant_handle());
}


DDS_InstanceHandle_t Datalink::participant_handle() const
{
    Q_D(const Datalink);
    Q_ASSERT(d);
    return d->participant->get_instance_handle();
}



#include "datalink.moc"
