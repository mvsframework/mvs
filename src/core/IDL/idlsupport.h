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

#ifndef IOSUPPORT_H
#define IOSUPPORT_H

#include <assert.h>
#include <iostream>
#include <QtCore/QObject>
#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtCore/QByteArray>

#include <ndds/ndds_cpp.h>

#include "IDL/data.h"

QString enumToQString( IDL::Data::StateDescriptorId const & stateId);
QString enumToQString(IDL::Data::NodeType const & nodeType);
QString enumToQString(IDL::Commands::CommandTypes const & commandType);


std::ostream& operator<<(std::ostream& out, IDL::Data::StateDescriptorId const & enumValue);
QDebug operator<<(QDebug dbg,  IDL::Data::StateDescriptorId const & enumValue);

std::ostream& operator<<(std::ostream& out, IDL::Commands::CommandTypes const & enumValue);
QDebug operator<<(QDebug dbg,  IDL::Commands::CommandTypes const & enumValue);

std::ostream& operator<<(std::ostream& out, IDL::Data::NodeType const & enumValue);
QDebug operator<<(QDebug dbg,  IDL::Data::NodeType const & enumValue);

std::ostream& operator<<(std::ostream& out, IDL::Data::Position_Datum const & position);
QDebug operator<<(QDebug dbg,  IDL::Data::Position_Datum const & position);


bool operator==( IDL::Data::Md5Hash const& lhs, IDL::Data::Md5Hash const& rhs);
bool operator!=( IDL::Data::Md5Hash const& lhs, IDL::Data::Md5Hash const& rhs);


IDL::Data::NodeType qStringToNodeTypeEnum(QString const & typeName);

namespace IDL {

QByteArray toQByteArray( Data::Md5Hash md5Hash );
Data::Md5Hash toMd5Hash ( QByteArray byteArray );

} // end namespace IDL


#endif // IOSUPPORT_H
