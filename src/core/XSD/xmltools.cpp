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

#include "xmltools.h"


namespace XML {
  
QDomElement getFirstTagElement(const QString& xmlTargetTag, const QDomDocument& document)
{
  QDomElement targetElement;
  QDomNodeList elementList = document.elementsByTagName(xmlTargetTag);
  if( 1 == elementList.size() )
  {
    targetElement = elementList.item(0).toElement();
  }
  else if( 1 < elementList.size() )
  {
    qWarning() << "The xmlTagetTag" << xmlTargetTag << "is not unique,"
      << elementList.size() << "were found. Using the first occurance..."; 
    targetElement = elementList.item(0).toElement();
  }
  else
  {
    qCritical() << "The xmlTagetTag" <<  xmlTargetTag << "was not found. Aborting...";
  }
  
  return targetElement;
};  
  
bool loadXmlFile(const QString& fileName, QDomDocument& document)
{
  QString errorMsg = "File does not seem to exist.";
  
  if( QFile::exists(fileName) )
  {
    QFile file(fileName);
     
    // load the file into the handed in XML document
    if( document.setContent(&file,&errorMsg) )
    {
      return true;
    }
    else
    {
      qCritical("XML Tools: Couldn't load XML data from %s.",fileName.toStdString().c_str());  
    }
  }
  
  qCritical() << "Can't seem to load" << fileName << ".\nError:" << errorMsg;
  return false;
}

bool loadXmlFile(const std::string& fileName, QDomDocument& document)
{
 QString name(fileName.c_str());
 return loadXmlFile(name,document);
}
  
  
} // end namespace XML