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

#ifndef GUSTDEBUGANDERRORMESSAGES_H
#define GUSTDEBUGANDERRORMESSAGES_H

#include "GustInterface/datalinkmessages.h"

#include <QtCore/QDebug>

QString enumToQString(const GUST::AltitueMode& enumValue);
QString enumToQString(const GUST::ButtonMap& enumValue);
QString enumToQString(const GUST::DataLinkMessageId& enumValue);
QString enumToQString(const GUST::HeadingMode& enumValue);
QString enumToQString(const GUST::ManeuverType& enumValue);
QString enumToQString(const GUST::TrajectoryCommand& enumValue);
QString enumToQString(const GUST::TrajectoryStatus& enumValue);
QString enumToQString(const GUST::M1_Safemode_Status& enumValue);



QDebug operator<<(QDebug dbg, const GUST::datalinkMessageFlightPlan_ref& m);
QDebug operator<<(QDebug dbg, const GUST::datalinkMessageFlightPlanWaypoint_ref& m);

#endif // GUSTDEBUGANDERRORMESSAGES_H
