/*
 *  Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
 * 
 *
 */

#ifndef TRANSITIONS_H
#define TRANSITIONS_H

#include <QtCore/QStateMachine>
#include <QtCore/QAbstractTransition>
#include <QtCore/QSignalTransition>

#include "StateMachine/events.h"

namespace Transitions {
    
  class CommandTransition : public QAbstractTransition
  {
  public:
    CommandTransition( const IDL::Commands::CommandTypes& command )
      :command (command)
    {}
    
  protected:
    virtual bool eventTest(QEvent* event) /*const*/
    {
      if( event->type() != QEvent::Type(Events::Types::CommandType) )
        return false;
      auto castEvent = static_cast<Events::UserCommand*>(event);
//         qDebug() << "CommandTransition::eventTest/*()*/ : Type Match";
      return command == castEvent->command;
    }
    
    virtual void onTransition(QEvent* event){}
    
  private:
    IDL::Commands::CommandTypes command;

  };
  
  /** \brief Transition on a certain GUST trajectoryStatus.
    * \todo GUST could change the transition status for all kinds of reasons, hence
    *    there needs to be the possibility to add some more conditions, e.g. the
    *    distance to a certain waypoint.
    */
  class TrajectoryTransition : public QAbstractTransition
  {
  public:
    TrajectoryTransition(const GUST::TrajectoryStatus& trajStatus)
      :trajectoryStatus(trajStatus)
    {}
    
  protected:
    virtual bool eventTest(QEvent* event) /*const*/
    {
      if( event->type() != QEvent::Type(Events::Types::GustTrajectoryType) )
        return false;
      auto castEvent = static_cast<Events::GustTrajectory*>(event);
//         qDebug() << "TrajectoryTransition::eventTest() : Type Match";
      return trajectoryStatus == castEvent->status;
    }
    
    
    virtual void onTransition(QEvent* event){}
  private:
    GUST::TrajectoryStatus trajectoryStatus;
  };
        
  class GuidanceTransition : public QAbstractTransition
  {
  public:
    GuidanceTransition(const GUST::M1_Safemode_Status& guidanceStatus)
      :guidanceStatus(guidanceStatus)
    {}
    
  protected:
    virtual bool eventTest(QEvent* event) /*const*/
    {
      if( event->type() != QEvent::Type(Events::Types::GustGuidanceType) )
        return false;
      auto castEvent = static_cast<Events::GustGuidance*>(event);
//         qDebug() << "TrajectoryTransition::eventTest() : Type Match";
      return guidanceStatus == castEvent->status;
    }
    
    
    virtual void onTransition(QEvent* event){}
  private:
    GUST::M1_Safemode_Status guidanceStatus;
  };
    
    
}; // end namespace Transitions

#endif // TRANSITIONS_H