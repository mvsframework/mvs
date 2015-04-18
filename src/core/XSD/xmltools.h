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

#ifndef XMLTOOLS_H
#define XMLTOOLS_H

#include <QtCore/QStringList>
#include <QtCore/QFile>
#include <QtCore/QDebug>

#include <QtXml/QDomDocument>

#include <string>


namespace XML {
  
/** \brief Splitting \<list itemType="xsd:string" \> XML elements.
 * 
 * This function splits space (" ") separated XML list items into a QList.
 * If a delimiter != " " is given, the seperated items are treated as a string 
 * and further separated at the delimiter character.
 * 
 * \tparam XmlList The C++ class type of the XML representing instance.
 * \param[in] element A C++ class instance representing the XML list element
 * \param[in] delimiter (optional) The delimiting character for further splitting.
 * \return The individual items as QStrings
 */
template<typename XmlList>
QList< QStringList > splitXmlStringList(XmlList element, const QString delimiter = ",")
{
  // The Coordinates_t type is a <list itemType="xs:string" />, hence the 
  // trouble with the iterators.
  // Compare:
  // http://www.codesynthesis.com/pipermail/xsd-users/2007-February/000816.html
  // http://www.codesynthesis.com/pipermail/xsd-users/2007-October/001295.html
  //
  // KML specifies its coordinates as a string list:
  // <coordinates>lon,lat,alt lon,lat,alt ...</coordinates>
  // The XML list type is delimited by the "space" in between the 3-tupels of 
  // individual coordinates for a 3D location...
  
  QList<QStringList> splitCoordinateGroups;
  
  // iterating over the space seperated groups...
  typename XmlList::iterator it_list = element.begin();
  while( it_list != element.end() )
  { 
    if( delimiter != NULL )
    { // split them at the delimiter
      splitCoordinateGroups.append( QString(std::string(*it_list).c_str()).split(delimiter) );
    }
    else
    { // append without further splitting
      splitCoordinateGroups.append( QStringList(QString(std::string(*it_list).c_str())) );
    } 
    ++it_list;
  }
  
  return splitCoordinateGroups;
};



/** \brief Returns the first element with the given tag name.
 * 
 * Also does some error checking and outputs those via qWarning() or 
 * qCritical(), respectively.
 * 
 * \param[in] xmlTagetTag The name of the tag to look for.
 * \param[in] document Where to look for the tag.
 * \return The first found occurance \b or an empty element.
 */
QDomElement getFirstTagElement(const QString& xmlTargetTag, const QDomDocument& document);


/** \brief Loads a XML file (with some error checking).
 * 
 * \param[in] fileName The (qualified) name of the file.
 * \param[out] document The container the file is loaded into.
 * \return true if loading was sucessful.
 */
bool loadXmlFile(const QString& fileName, QDomDocument& document);


/** \brief Loads a XML file (with some error checking).
 * \overload
 * \param[in] fileName The (qualified) name of the file.
 * \param[out] document The container the file is loaded into.
 * \return true if loading was sucessful.
 */
bool loadXmlFile(const std::string& fileName, QDomDocument& document);

template<typename XmlStringType>
std::string stdString(XmlStringType xmlString)
{
  return std::string(xmlString);
};

template<typename XmlStringType>
QString qString(XmlStringType xmlString)
{
  return QString(std::string(xmlString).c_str());
};

} // end namespace XML


#endif // XMLTOOLS_H
