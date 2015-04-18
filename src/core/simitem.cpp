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

#include "simitem.h"
#include "simitem_p.h"

#include "exceptions.h"
#include "IDL/data.h"
#include "StateMachine/state.h"
#include "XSD/xmltools.h"

//############################################################################//
//                                                                            //
//  SimItemPrivate                                                            //
//                                                                            //
//############################################################################//

SimItemPrivate::SimItemPrivate( SimItem* q, SimItem::Settings* settings )
  :q_ptr( q )
  ,xmlSettings(settings)
{//NOTE: Do _NOT_ use the q_ptr in here, it _WILL_ break things!
//   qDebug() << "SimItemPrivate()";


}

SimItemPrivate::~SimItemPrivate()
{
  Q_Q(SimItem); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": ~SimItemPrivate()";
}

bool SimItemPrivate::isStatusChangePossible(const SimItem::Status& newStatus) const
{
  switch( currentStatus )
  {
    case SimItem::Status::Undefined: // currentStatus
      switch( newStatus )
      {
        case SimItem::Status::Undefined: 
        case SimItem::Status::Constructed: 
          return true;
        case SimItem::Status::Initialized:
        case SimItem::Status::Running:
        default: return false;
      }
    case SimItem::Status::Constructed: // currentStatus
      switch( newStatus )
      {
        case SimItem::Status::Constructed: 
        case SimItem::Status::Initialized:
          return true;
        case SimItem::Status::Undefined: 
        case SimItem::Status::Running:
        default: return false;
      }
    case SimItem::Status::Initialized: // currentStatus
      switch( newStatus )
      {          
        case SimItem::Status::Initialized:
        case SimItem::Status::Running:
          return true;
        case SimItem::Status::Undefined: 
        case SimItem::Status::Constructed: 
        default: return false;
      }
    case SimItem::Status::Running: // currentStatus
      switch( newStatus )
      {
        case SimItem::Status::Initialized:
        case SimItem::Status::Running:          
          return true;
        case SimItem::Status::Undefined: 
        case SimItem::Status::Constructed: 
        default: return false;
      }
    default:return false;
  }
}

void SimItemPrivate::setStatus(const SimItem::Status& newStatus)
{
  Q_Q(SimItem); Q_ASSERT(q);
  qDebug() << q->qualifiedName() << ": Status change" << currentStatus << "-->" 
  << newStatus;

  currentStatus = newStatus;
  emit q->statusChanged(currentStatus);
}





void SimItemPrivate::initializeStateMachine()
{
  Q_Q(SimItem); Q_ASSERT(q);
  
  // If this SimItem is a root item, then create a new state machine, 
  // otherwise piggyback on the parent's stateMachine.
  if( q->parent() == nullptr )
  { 
    stateMachine = new StateMachine(q);
    stateMachine->setObjectName(q->tr("State Machine"));
    
    // 
    // create the root state
    // =====================
    // 
    running = new State(IDL::Data::STATE_ID_I_RUNNING ,QState::ParallelStates,nullptr); 
    stateMachine->addState(running);
    stateMachine->setInitialState(running);
    
  }
  else
  {
    stateMachine = q->parent()->stateMachine();
    running = dynamic_cast<State*>(stateMachine->initialState());
    Q_ASSERT(running);
  }; 
}


void SimItemPrivate::initializeStates()
{
  Q_Q(SimItem); Q_ASSERT(q);
  using namespace IDL::Data;
   
  //
  // create the states
  // =================
  // Some of the created states would normally not be necessary when drawing
  // a statechart, but are required because of how the Qt QStateMachine works.
  // Those states are marked as "A Qt necessity" and most often represent 
  // orthogonal regions.
  //

  running_itemRegion = new State(STATE_ID_I_RUNNING_REGION ,running); // A Qt necessity
  
  QString stateName = q->objectName().prepend("(").append(" Region)");
  running_itemRegion->setObjectName(stateName);
  
  //
  // set the initial (sub-)states
  // ==============================
  // Any "Region" state is "just" a Qt construct to allow parallelization at 
  // the level directly above. As such regions don't need to be set themselves 
  // as initial states of anything as their use implies that they are children 
  // of a QState::ParallelStates compound state and Qt automatically enters all
  // parallel children of compound states upon entry.
  //
  // NOTE: Activities automatically set the "done" substate as their initial 
  // state.
  //
  
  {}
  
  //
  // add the transitions
  // ===================
  // NOTE: When creating custom transitions (i.e. subclassing QAbstractTransition)
  //   do remember that a state to which a transition is added assumes possession
  //   of that transition pointer. As a result, a transiotn pointer cannot be
  //   reused!!
  //
}


void SimItemPrivate::unInitializeSimItem()
{
  Q_Q(SimItem); Q_ASSERT(q);
//   qDebug() << q->qualifiedName() << ": SimItemPrivate::unInitializeSimItem()";
  
  if( q->isInitialized() ) // i.e. not only constructed.
  {

//NOTE:  We don't need any of this as the state machine is a (QObject) child of 
// this (QObject), hence Qt takes care of the proper deletion.
//
//     // un-initialize stateMachine
//     if(     stateMachine != nullptr
//         and stateMachine->parent() == dynamic_cast<QObject*>(this))
//     { 
//       stateMachine->deleteLater();
//       stateMachine = nullptr;
//     }

  }
}













//############################################################################//
//                                                                            //
//  SimItem                                                                   //
//                                                                            //
//############################################################################//


/** \note This constructor does NOT do any work!
 * All Node level construction needs to happen in 
 * SimItem(SimItemPrivate& dd, ...)!
 */
SimItem::SimItem(Settings * const settings, SimItem* parent)
  :SimItem(*new SimItemPrivate(this,settings),parent)
{}



SimItem::SimItem(SimItemPrivate& dd, SimItem* parent)
  :QObject(parent)
  ,d_ptr(&dd) //NOTE: it should be safe to set the d_ptr as at this time as 
              // SimItemPrivate should be fully constructed
{
  Q_D(SimItem); Q_ASSERT(d);
  if( d->xmlSettings != nullptr )
  {
    setObjectName(XML::qString(d->xmlSettings->name()));
  }
  
  qDebug() << qualifiedName() << ": SimItem(SimItemPrivate& dd, SimItem* parent)"
    << "with parent =" << parent
    << "and thread affinity" << thread();
} 

SimItem::~SimItem()
{
//   qDebug() << qualifiedName() << ": ~SimItem()";
  Q_D(SimItem); Q_ASSERT(d);
  if(status() >= Status::Initialized)
  { 
    if( status() >= Status::Running )
    { halt(); };
    d->unInitializeSimItem();
  }
}

SimItem* SimItem::parent() const
{ return dynamic_cast<SimItem*>(QObject::parent()); }


QString SimItem::qualifiedName() const
{
  QString fullName = objectName();
  
  for( SimItem* p = parent(); p!=nullptr ; p = p->parent() )
  { fullName.prepend(p->objectName().append("/")); };
  
  return fullName;
}



SimItem::Status SimItem::status() const
{
  Q_D(const SimItem); Q_ASSERT(d);
  return d->currentStatus;
}

bool SimItem::isInitialized()
{ return status() >= Status::Initialized; }

bool SimItem::isRunning()
{ return status() >= Status::Running; }


StateMachine* SimItem::stateMachine()
{
  Q_D(SimItem); Q_ASSERT(d);
  return d->stateMachine;
}

// const StateMachine* SimItem::stateMachine() const
// {
//   Q_D(const SimItem); Q_ASSERT(d);
//   return d->stateMachine;
// }



void SimItem::initialize()
{ 
  Q_D(SimItem); Q_ASSERT(d);
  if( d->isStatusChangePossible(Status::Initialized) )
  {
    qDebug() << qualifiedName() << ": SimItem::initialize()";
//     try
//     {
      if(status() >= Status::Initialized)
      { unInitializeItem(); }
    
      initializeItem();
      
//NOTE: Although a nice idea, this fails at some relationsships as some parents
// need their children initialized first (e.g. Node needs Datalink to be up to
// initialize...)
//      
//       SimItem* child;
//       foreach(child, findChildren<SimItem*>())
//       { child->initialize(); }
      
      d->setStatus(Status::Initialized);
//     }
//     catch(std::runtime_error & e)
//     {
//       Q_CHECK_PTR( e.what() );
//       qFatal("\"%s\" : Initialization failed. (what(): %s)"
//         ,qualifiedName().toStdString().c_str() 
//         ,e.what() ) ;
//     }
  }
  else
  {
    qCritical() << qualifiedName()
    << ": Cannot (re-)initialize, currentStatus ==" << d->currentStatus;
  }
}

void SimItem::run()
{ 
  Q_D(SimItem); Q_ASSERT(d);
  if( d->isStatusChangePossible(Status::Running) )
  {
    qDebug() << qualifiedName() << ": SimItem::run()";
    startItem(); 

//NOTE: Not certain if this is really a good idea...
//         
//     SimItem* child;
//     foreach(child, findChildren<SimItem*>())
//     { child->run(); }
    
    d->setStatus(Status::Running);
  }
  else
  {
    qCritical() << qualifiedName()
    << ": Cannot start, currentStatus ==" << d->currentStatus;
  }
}

void SimItem::halt()
{ 
  Q_D(SimItem);
  Q_ASSERT(d);
  if( d->isStatusChangePossible(Status::Initialized) )
  {

//NOTE: Not certain if this is really a good idea...
//       
//     SimItem* child;
//     foreach(child, findChildren<SimItem*>())
//     { child->halt(); }
    qDebug() << qualifiedName() << ": SimItem::halt()";
    stopItem(); 
    
    d->setStatus(Status::Initialized);
  }
  else
  {
    qCritical() << qualifiedName()
    << ": Cannot stop, currentStatus ==" << d->currentStatus;
  }
}

void SimItem::initializeItem()
{
  Q_D(SimItem); Q_ASSERT(d);
  
  { /// - initialize the state machine    
    d->initializeStateMachine();
    d->initializeStates();
  }
}

void SimItem::startItem()
{
  Q_D(SimItem); Q_ASSERT(d);
  d->stateMachine->start();
}

void SimItem::stopItem()
{ 
  Q_D(SimItem); Q_ASSERT(d);
  d->stateMachine->stop();
}

void SimItem::unInitializeItem()
{
//    qDebug() << qualifiedName() << ":SimItem::unInitializeItem())";
   Q_D(SimItem); Q_ASSERT(d);
   d->unInitializeSimItem();
}



//############################################################################//
//                                                                            //
//  I/O Support                                                               //
//                                                                            //
//############################################################################//


QString enumToQString(SimItem::Status const & status)
{
  switch( status )
  {
    case SimItem::Status::Undefined :   return SimItem::tr("Undefined");
    case SimItem::Status::Constructed : return SimItem::tr("Constructed");
    case SimItem::Status::Initialized : return SimItem::tr("Initialized/Stopped");
    case SimItem::Status::Running :     return SimItem::tr("Running");

    default: return QObject::tr("(Unknown SimItem::Status)");
  }
};

std::ostream& operator<<(std::ostream& out, SimItem::Status const & status)
{ return out << enumToQString(status).toStdString(); }


QDebug operator<<(QDebug dbg, SimItem::Status const & status)
{
  dbg.nospace() << enumToQString(status);
  return dbg.space();
}



#include "simitem.moc"