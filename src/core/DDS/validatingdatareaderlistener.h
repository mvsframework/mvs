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

#ifndef VALIDATINGDATAREADERLISTENER_H
#define VALIDATINGDATAREADERLISTENER_H

#include <ndds/ndds_cpp.h>
#include <QtCore/QtGlobal>

#include <QtCore/QDebug>
#include <iostream>

class SimItem;

/** \brief A base class for specialized DataReader Listeners.
 * 
 * \internal
 * I wanted to make this a QObject but ran into construction issues as the
 * listener, mainly beeing used in the ...Private sections, would be created
 * as a child of a not yet fully constructed parent.
 * 
 * This class extends the basic DDSDataReaderListener. It implements all
 * callbacks for all status masks, but keeps them virtual for adapted handling
 * down the chain.
 * 
 * \note The simple presence of the callbacks doesn't mean that they indeed are
 * triggered. Check the DDS_StatusMask associated with each listener to ensure 
 * that all callbacks implemented are actually triggered, too.
 * This normally is handled by a reader->set_listener(listener,mask) routine.
 * 
 * The main thing this class does is to provide another layer of indirection 
 * and simplification for the on_data_available( DDSDataReader* reader ) 
 * callback: inside that function the received sample is checked and, if all is
 * OK, on_valid_data_available is called. This allows to skip a whole lot of
 * redundant typing when implementing listeners.
 * 
 * The class also provides for a SimItem* parent(), returning data set during
 * construction.
 * 
 * \tparam NarrowReader_t The type of the related narrowed DataReaderListener
 * \tparam Sequence_t     The type of the topic data sequence
 * \tparam Data_t         The type of the data in Sequence_t (somewhat redundant)
 * 
 */
template< typename NarrowReader_t 
        , typename Sequence_t     
        , typename Data_t
        , typename Parent_t
        >
class ValidatingDataReaderListener : public DDSDataReaderListener {
public:
  ValidatingDataReaderListener(Parent_t* const parent = nullptr)
    :m_parent( parent )
  {};
  virtual ~ValidatingDataReaderListener()
  {};
  
  /** \brief A status mask capturing all implemented callback routines. */
  static const DDS_StatusMask implementedCallbacks = 0
    | DDS_REQUESTED_DEADLINE_MISSED_STATUS 
    | DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS 
    | DDS_SAMPLE_REJECTED_STATUS 
    | DDS_LIVELINESS_CHANGED_STATUS 
    | DDS_SAMPLE_LOST_STATUS 
    | DDS_SUBSCRIPTION_MATCHED_STATUS
    | DDS_DATA_AVAILABLE_STATUS
    ;
   
  
  
  virtual void on_requested_deadline_missed( DDSDataReader* reader,
   DDS_RequestedDeadlineMissedStatus const & status)
  { 
    Q_UNUSED(reader); Q_UNUSED(status); 
    qWarning() << parent()->qualifiedName() << "(ValidatingDataReaderListener)"
      << ": DDS requested deadline missed.";
  };
   
  virtual void on_requested_incompatible_qos( DDSDataReader* reader,
    DDS_RequestedIncompatibleQosStatus const & status)
  { 
    Q_UNUSED(reader); Q_UNUSED(status); 
    qWarning() << parent()->qualifiedName() << "(ValidatingDataReaderListener)"
      << ": DDS incompatible QoS requested.";
  };
  
  virtual void on_sample_rejected( DDSDataReader* reader,
    DDS_SampleRejectedStatus const & status)
  { 
    Q_UNUSED(reader); Q_UNUSED(status); 
    qWarning() << parent()->qualifiedName() << "(ValidatingDataReaderListener)"
      << ": DDS sample rejected.";
  };

  virtual void on_liveliness_changed( DDSDataReader* reader,
    DDS_LivelinessChangedStatus const & status)
  {  Q_UNUSED(reader); Q_UNUSED(status);
//     qDebug() << parent()->qualifiedName() << "(ValidatingDataReaderListener)"
//       << ": Liveliness changed for topic" << reader->get_topicdescription()->get_name() << "\n"
//       << "    alive count:" << status.alive_count << "(change:"<< status.alive_count_change << ")\n"
//       << "not alive count:" << status.not_alive_count << "(change:"<< status.not_alive_count_change << ")";
  };

  virtual void on_sample_lost( DDSDataReader* reader,
    DDS_SampleLostStatus const & status)
  { 
    Q_UNUSED(reader); Q_UNUSED(status); 
    qWarning() << parent()->qualifiedName() << "(ValidatingDataReaderListener)"
      << ": DDS sample lost.";
  };

  virtual void on_subscription_matched( DDSDataReader* reader,
    DDS_SubscriptionMatchedStatus const & status)
  { Q_UNUSED(reader); Q_UNUSED(status); };

  virtual void on_data_available( DDSDataReader* reader )
  {
    NarrowReader_t*     builtin_reader = nullptr;
    Sequence_t          data_seq;
    DDS_SampleInfoSeq   info_seq;
    DDS_ReturnCode_t    retcode;
//     DDS_ExceptionCode_t ex;
    int                 i = 0;

    builtin_reader = NarrowReader_t::narrow(reader);
    if (builtin_reader == nullptr)
    {
      qCritical() << parent()->qualifiedName() << ": DDS BuiltinDataReader narrow error";
      return;
    }

    retcode = builtin_reader->take(
        data_seq, info_seq, DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);

    if (retcode == DDS_RETCODE_NO_DATA)
    { return; }
    else if (retcode != DDS_RETCODE_OK)
    {
      qCritical() << parent()->qualifiedName() << ": DDS take error:"<< retcode;
      return;
    }

    for (i = 0; i < data_seq.length(); ++i)
    {
      DDS_SampleInfo& info = info_seq[i];
      if (info.valid_data)
      {
        /** \internal I used to check that the sender is not the same
          * as the own (i.e. callback) node by calling 
          * 
          * <tt>if( callbackNode->id() != senderId  ){...}</tt>
          * 
          * However, I finally found the proper way to discard all messages
          * from the "local" participant's writers: 
          * DDDSDomainParticipant::ignore_participant()
          * See http://community.rti.com/kb/how-do-i-get-datareader-ignore-datawriter-belongs-same-domainparticipant
          * for an example.
          * 
          * The corresponding call to ignore_participant is done in 
          * DatalinkPrivate::initializeDatalink().
          */
        Data_t& data = data_seq[i];
        on_valid_data_available(data,info,builtin_reader);  
      }
      else
      {
        on_sample_info_available(info, builtin_reader); 
      }
    }

    retcode = builtin_reader->return_loan(data_seq, info_seq);
    if (retcode != DDS_RETCODE_OK)
    { qCritical() << parent()->qualifiedName() <<": return loan error" << retcode; }      
    
  };
  
protected:
 
  Parent_t* parent() const
  { return m_parent; };
  
  virtual void on_valid_data_available(Data_t & data, DDS_SampleInfo & info,NarrowReader_t * reader)=0  ;
  
  virtual void on_sample_info_available(DDS_SampleInfo & info, NarrowReader_t * reader)
  {    
//   Print some debug messages.
    QString participantDetails;
    if(info.instance_state == DDS_ALIVE_INSTANCE_STATE )
    { participantDetails = "(New)"; }
    else if( info.instance_state == DDS_NOT_ALIVE_DISPOSED_INSTANCE_STATE )
    { participantDetails = "(Dissapeared - Deleted)"; }
    else if( info.instance_state == DDS_NOT_ALIVE_NO_WRITERS_INSTANCE_STATE )
    { participantDetails = "(Dissapeared - Lost Connection)"; }
    else
    { /* there are no other options in DDS_InstanceStateKind. */}
    
//     qDebug() << "Participant" << participantDetails <<"\n"
//         << "messageNum:" << info.reception_sequence_number.low << "\n"
//         << "created at:" << info.source_timestamp << "\n"
//         << "detected at:" << info.reception_timestamp << "\n"
//         << "source sequence number:" << info.publication_sequence_number.low << "\n"
//         << "handle:" << info.instance_handle;
  };
  
private:
  Parent_t* const m_parent;

};

#endif // VALIDATINGDATAREADERLISTENER_H
