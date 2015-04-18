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

#ifndef TOPIC_H
#define TOPIC_H



#include <ndds/ndds_cpp.h>

#include "exceptions.h"
#include "DDS/ddssupport.h"
#include "DDS/validatingdatareaderlistener.h"
#include "DDS/topicoptions.h"
#include "DDS/datalink.h"
#include "Nodes/node.h"

#define TOPIC_CAN_WRITE (options == TopicOptions::Send    or options == TopicOptions::SendReceive)
#define TOPIC_CAN_READ  (options == TopicOptions::Receive or options == TopicOptions::SendReceive)

/** \brief A container for DDS Topic related entities.
 * 
 * \tparam Message The IDL::Messages type this topic is using.
 * \tparam ParentNodeType The Node derived class that introduces this topic.
 * \tparam options The TopicOptions for this Topic.
 */
template<typename Message
        ,typename ParentNodeType
        ,TopicOptions options
        >
class Topic {

//   using Message       = Message;
  using Sequence    = typename Message::Seq;
  using TypeSupport = typename Message::TypeSupport;
  using DataWriter  = typename Message::DataWriter;
  using DataReader  = typename Message::DataReader;

  
public:
  /** \brief (Delegating) Constructor. */
  Topic(QString const & name, Datalink* const  datalink, 
        ParentNodeType* const callbackNode)
   :Topic(name ,datalink->participant() ,datalink->publisher()
         ,datalink->subscriber() ,callbackNode)
  {};
  
  /** \brief Destructor. */
  virtual ~Topic()
  {     
    
    if( options == TopicOptions::Send or options == TopicOptions::SendReceive )
    {
      publisher->delete_datawriter(dataWriter);
      dataWriter = nullptr;
    }
    
    if( options == TopicOptions::Receive  or options == TopicOptions::SendReceive )
    {
      delete readerListener;
      readerListener = nullptr;
      
      dataReader->delete_contained_entities(); // delete all ReadConditions and QueryConditions
      
      subscriber->delete_datareader(dataReader);
      dataReader = nullptr;
    }
    
    participant->delete_topic(topic);
    topic = nullptr;
    
    {/// - Unregister the message type after deleting the DDS topic
      typeName = TypeSupport::get_type_name();
      auto retcode  = TypeSupport::unregister_type( participant, typeName.toStdString().c_str());
      if (retcode != DDS_RETCODE_OK)
      { 
        std::cerr << "unregister_type error " << retcode << std::endl;
        throw datalink_error("unregister_type error",retcode);
      }
    }
  };
  
  /** \brief Get the name of the Topic. */
  QString name() const
  {return topicName; };
  
  /** \brief Get the data writer for this Topic. */
  DataWriter* writer()
  { 
    if( TOPIC_CAN_WRITE )
    {
      Q_ASSERT(dataWriter);
      return dataWriter;
    }
    qCritical() << "Requesting a data writer from a non-writing topic.";
    return nullptr;
  };
  
  /** \brief Get the data reader for this Topic. */
  DataReader* reader()
  {
    if( TOPIC_CAN_READ )
    {
      Q_ASSERT(dataReader);
      return dataReader;
    }
    qCritical() << "Requesting a data reader from a non-reading topic.";
    return nullptr;
  };
    
  /** \brief Publish a message for this topic.
   * \param[in] message The message data to send.
   * \param[in] instanceHandle (optional) The handle for the keyed message.
   * \return TRUE if the sending was successful; FALSE otherwise.
   */
  virtual bool send(Message message, DDS_InstanceHandle_t instanceHandle = DDS_HANDLE_NIL) const
  {
    if( !isInitialized() )
    { 
      qWarning() << "Topic Error: Initialize" << name() << "before attempting to send messages.";
      return false;
    }
    
    if( TOPIC_CAN_WRITE )
    {
      
      /** \internal From the RTI User's Manual:
       * "If there are matched DataReaders, then the data sample will have been 
       * passed to the physical transport plug-in/device driver by the time that 
       * write() returns." */
      
      auto retcode  = dataWriter->write(message, instanceHandle);
      if( retcode != DDS_RETCODE_OK)
      { 
        qCritical() << "DDS Error: dataWriter error in Topic" << name() 
        << ". RETCODE = " << retcode ; 
      }; 
      return retcode == DDS_RETCODE_OK;
    }
    // else
    return false;
  }
   
  /** \brief Delete an instance of Message and free the related memory.
   * 
   * This is a (deep) verision of <tt> delete message;</tt>, which <b> should
   * not be used</b>!
   * 
   * \param[in] data The pointer to the instanec to be deleted.
   * \return The DDS return code for the operation.
   */
  static DDS_ReturnCode_t deleteMessageStruct(Message* data)
  { return TypeSupport::delete_data(data); };
  
  /** \brief Create a new instance of Message.
   * 
   * This is essentially comparable to a <tt>new Message;</tt> command where 
   * Connext takes care of the (deep) instantiation of message structs.
   * 
   * \note The caller takes responsibility for the newly allocated memory! (I.e.
   * remeber to call deleteMessageData().)
   * 
   * \return A pointer to the newly allocated instance of Message.
   */
  static Message* createMessageStruct()
  { return TypeSupport::create_data(); };
  
  
  /** \brief Initialize the Topic and create the DataReader and DataWriter.
   * \details After a Topic is initialized, the QoS settings for the reader and
   * cannot be changed any longer. */
  void initialize()
  {
    if( isInitialized() )
    {
      qWarning() << "Topic error: Topic" << name() << "cannot be (re-)initialized.";
      return;
    }   
    
    {/// Create the actual DDS topic entitiy
      topic = participant->
        create_topic(topicName.toStdString().c_str() // Topic Name
                    ,typeName.toStdString().c_str()  // Type Name
                    ,DDS_TOPIC_QOS_DEFAULT           // QoS
                    ,nullptr                         // Listener
                    ,DDS_STATUS_MASK_NONE            // Listener Mask
                    );          
      Q_ASSERT(topic);
    }    
      
      
    if( TOPIC_CAN_WRITE )
    {/// Create the dataWriter entity
    
      DDS_DataWriterQos dataWriter_qos;
      DDS_ReturnCode_t retcode = publisher->get_default_datawriter_qos(dataWriter_qos);
      if (retcode != DDS_RETCODE_OK )
      { qCritical() << "DDS error:" << retcode; }  
    
      modifyWriterQos(dataWriter_qos);
    
      DDSDataWriter* generic_writer = publisher->
        create_datawriter(topic                      // Topic
                         ,dataWriter_qos             // QoS
                         ,nullptr                    // Listener
                         ,DDS_STATUS_MASK_NONE       // Listener Mask
                         );
      Q_ASSERT(generic_writer);
    
      dataWriter = DataWriter::narrow(generic_writer);
      Q_ASSERT(dataWriter);
    }
    
    if( TOPIC_CAN_READ )
    {/// Create the dataReader entity
          
      DDS_DataReaderQos dataReader_qos;
      DDS_ReturnCode_t retcode = subscriber->get_default_datareader_qos(dataReader_qos);
      if (retcode != DDS_RETCODE_OK )
      { qCritical() << "DDS error:" << retcode; }  
      modifyReaderQos(dataReader_qos);   
    
      readerListener = new ReaderListener( callbackNode );
      
      DDSDataReader* generic_reader = subscriber->
        create_datareader(topic                     // Topic
                         ,dataReader_qos            // QoS
                         ,readerListener            // Listener
                         ,ReaderListener::implementedCallbacks // Listener Mask
                         );
      Q_ASSERT(generic_reader);
      
      dataReader = DataReader::narrow(generic_reader);
      Q_ASSERT(dataReader);
      
//       QObject::connect(readerListener, &ReaderListener::dataAvailable(Node::Id, Message),
//                        callbackNode,  &Node::processReceivedMessage(Node::Id,Message));   
    }
    
  };
  
  
private:
  Q_DISABLE_COPY(Topic)
  
    /** \brief Constructor. */
  Topic(QString const & name
       ,DDSDomainParticipant* const participant
       ,DDSPublisher* const publisher
       ,DDSSubscriber* const subscriber
       ,ParentNodeType* const callbackNode
       )
    :topicName(name)
    ,callbackNode(callbackNode)
    ,participant(participant)
    ,publisher(publisher)
    ,subscriber(subscriber)
  {
    Q_ASSERT( participant );
    Q_ASSERT( publisher );
    Q_ASSERT( subscriber );
    Q_ASSERT( callbackNode );
    
    {/// Register the message type before creating a DDS topic
    
      //NOTE: I added some //@copy-declaration instructions into messages.idl 
      // to call Q_DECELARE_METATYPE() to make this work. 
      qRegisterMetaType<Message>();
    
      typeName = TypeSupport::get_type_name();
      auto retcode  = TypeSupport::register_type( participant, typeName.toStdString().c_str());
      if (retcode != DDS_RETCODE_OK)
      { 
        qCritical() << "DDS error: register_type error:" << retcode;
        throw datalink_error("register_type error",retcode);
      }
    }
    
  };
  
  
 
  bool isInitialized() const
  { return topic!=nullptr; };
  
  
  virtual void modifyWriterQos(DDS_DataWriterQos & qos)
  {        
//       QHash<QString,QString> props;
//       props["id"]= QString("42");
//       props["node"] = QString("23");
//       DDS::appendHashToProperty(props, qos.property.value);
  };
  
  virtual void modifyReaderQos(DDS_DataReaderQos & qos){};
  
  
  class ReaderListener : /*public QObject,*/ public ValidatingDataReaderListener<DataReader,Sequence,Message,ParentNodeType>
  {
//   Q_OBJECT
  
//   Q_SIGNALS:
//     void dataAvailable(Node::Id senderId, Message data);
  
  public:
    
    using Base = ValidatingDataReaderListener<DataReader,Sequence,Message,ParentNodeType>;
    
    /** \brief A status mask capturing all implemented callback routines. */
    static const DDS_StatusMask implementedCallbacks = Base::implementedCallbacks
      | DDS_LIVELINESS_CHANGED_STATUS
      ;
      
  public:
    
    ReaderListener(ParentNodeType* const node)
      :ValidatingDataReaderListener<DataReader,Sequence,Message,ParentNodeType>(node)
    {};
    
/*    virtual void on_liveliness_changed( DDSDataReader* reader,
      DDS_LivelinessChangedStatus const & status)
    { 
      
      qDebug() << parent()->qualifiedName() << "(ReaderListener)"
        << ": Liveliness changed for topic" << reader->get_topicdescription()->get_name() << "\n"
        << "    alive count:" << status.alive_count << "(change:"<< status.alive_count_change << ")\n"
        << "not alive count:" << status.not_alive_count << "(change:"<< status.not_alive_count_change << ")";
    }; */  
  protected:
    
    //NOTE: This 'using' declaration has to be here to bring the protected
    // function ValidatingDataReaderListener::parent() into scope.
    // I don't fully understand it, but is seems that this has something to do
    // with the two-phase lookup of template related things... (Sorry, still no
    // degree in CS...)
    using ValidatingDataReaderListener<DataReader,Sequence,Message,ParentNodeType>::parent;
   
    virtual void on_valid_data_available( Message& data,
                                          DDS_SampleInfo& info,
                                          DataReader* reader )
    {
      /** \internal I used to check that the sender is not the same
        * as the own (i.e. callback) node by calling 
        * 
        * <tt>if( callbackNode->id() != senderId  ){...}</tt>
        * 
        * (where the senderId was/is part of the message structure).
        * 
        * However, I finally found the proper way to discard all messages
        * from the "local" participant's writers: 
        * DDDSDomainParticipant::ignore_participant()
        * See http://community.rti.com/kb/how-do-i-get-datareader-ignore-datawriter-belongs-same-domainparticipant
        * for an example.
        */
      

      //NOTE: info_seq[i].publication_handle effectively is a reference to 
      // the DataWriter that sent the sample, encoded in a handle.         
      DDS_PublicationBuiltinTopicData publicationData;
      reader->get_matched_publication_data(publicationData,
                                          info.publication_handle);
    
      NodeId senderId(publicationData.participant_key);
      
      ParentNodeType* parentNode = parent();
      
      bool messageIsReceivable = true;

#ifdef WITH_RFSIMULATION      
      {/** \internal Compute whether or not the received message would actually
        * be receivable. This is the part where we simulate the environment, 
        * i.e. messages sent by nodes that are not part of the same connected
        * subset as the receiving node (in the comGraph) are discarded.
        */
        messageIsReceivable = parentNode->isMessageReceivable(senderId,data);
      }
#endif // WITH_RFSIMULATION
      
      if( parentNode->isRunning() and messageIsReceivable) //FIXME: halt() should terminate the topic.
      { 
        // direct function call... //TODO: Does this work in the given threading scenario?
        parentNode->processReceivedMessage(senderId, data); 
      }
      else
      { 
//         QString str;
//         if( parentNode->blipExists(senderId) )
//         { str = parentNode->blip(senderId).name(); }
//         else
//         { str =  senderId;}
//         qDebug() << parentNode->qualifiedName() << ":Topic \"" << reader->get_topicdescription()->get_name()
//         <<"\" is discarding a message from" << str; 
      }
      
    };

  };
    
  QString topicName;
  
  ParentNodeType* const callbackNode;
  
  /** \brief The DDS participant responsible for this TopicContainer. */
  DDSDomainParticipant* const participant;
  
  DDSPublisher* const publisher;
  DDSSubscriber* const subscriber;
  
  DataWriter* dataWriter = nullptr;
  DataReader* dataReader = nullptr;
  
  ReaderListener* readerListener = nullptr;
  
  QString typeName;
  
  /** \brief The actual DDS topic entity.*/
  DDSTopic* topic = nullptr; 
  
  Message const * defaultMessage = nullptr;
  DDS_InstanceHandle_t const * defaultInstanceHandle = nullptr;
  
};

#endif // TOPIC_H
