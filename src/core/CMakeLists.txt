# 
#   Copyright © 2015 Claus Christmann <hcc |ä| gatech.edu>.
# 
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
# 
#       http://www.apache.org/licenses/LICENSE-2.0
# 
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#


# set this directory as a search directory for includes
include_directories( ${CMAKE_CURRENT_LIST_DIR} )


#
# Deal with Qt5
# =============
#

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)


#
# Deal with RTI DDS
# =================
# RTI DDS generator settings
#
set(NDDS_RTIDDSGEN_FILE   "${BASE_DIR_NDDS}/scripts/rtiddsgen" 
  CACHE FILEPATH "The rtiddsgen binary to use to process IDL files."
)

set(NDDS_XSD_TARGET_DIR ${PROJECT_SOURCE_DIR}/XSD )
set(NDDS_RTIDDSGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/IDL)

# the list of raw IDL source files (.idl)
set(IDL_SRCS
  IDL/data.idl
  IDL/messages.idl
#   settings.idl
)

include(IDL/rtiddsgen.cmake)


#
# Deal with CodeSynthesis XSD
# ===========================
# 

# 
# CodeSynthesis XSD Generator settings
#
set( CODESYNTHESIS_XSD_FILE   "/usr/bin/xsdcxx"
  CACHE FILEPATH "The CodeSynthesis binary used to process XSD files."
)

set( CODESYNTHESIS_XSD_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/XSD)
set( CODESYNTHESIS_XSD_TARGET_DIR ${PROJECT_SOURCE_DIR}/XSD)


set( XSD_SRCS
#   ${NDDS_XSD_TARGET_DIR}/rti_dds_topic_types_common.xsd
  XSD/sai.xsd
)

include(XSD/codesynthesis.cmake)

# allow the inclusion of the generated files with #include "XSD/..."
include_directories(${CMAKE_CURRENT_BINARY_DIR})


# Sources
# =======
#

set( mvs_core_SRCS
  ${_rti_autogen_SRCS}
  IDL/idlsupport.cpp 
  ${_codesynthesis_autogen_SRCS}
  XSD/xmltools.cpp
  StateMachine/statemachine.cpp
  StateMachine/state.cpp 
  simitem.cpp
  template_deriveditem.cpp
  
  StateMachine/activity.cpp
  Geometry/geometry.cpp
  DDS/ddssupport.cpp
  DDS/datalink.cpp
  Environment/element.cpp
  Environment/obstacle.cpp
  Environment/path.cpp
  blip.cpp 
  vroniinterface.cpp
  Workers/comformationthread.cpp
  Nodes/node.cpp
  
  GNC/waypoint.cpp
  GNC/flightplan.cpp
  Nodes/vehicle.cpp
  
  GustInterface/debuganderrormessages.cpp
  GustInterface/checksum.cpp
  GustInterface/gustinterface.cpp
  Nodes/gustuav.cpp 

  HMI/joystick.cpp
  HMI/humanmachineinterface.cpp
  Nodes/controlstation.cpp

)

set(mvs_core_HDRS
  exceptions.h
  StateMachine/statemachine.h
  StateMachine/state.h
  simitem.h
  nodeid.h
  nodeid_128.h
  elementid.h
  blip.h
  DDS/ddssupport.h
  Environment/element.h
  Environment/obstacle.h
  Environment/path.h
  Geometry/geometry.h
  Geometry/point_2d.h
  Geometry/circle_2d.h
  Geometry/pie_2d.h
  Graph/comgraph.h
  Graph/routegraph.h
  Graph/vronigraph.h
  Graph/graphsupport.h
  Graph/properties.h
  DDS/topicoptions.h
  Nodes/node.h
  GNC/flightplan.h
  GNC/waypoint.h
  GustInterface/datalinkmessages.h
  Nodes/vehicle.h
  Nodes/gustuav.h
  Nodes/controlstation.h
  XSD/xmltools.h
)

set( mvs_core_GRAPHICS_SRCS
  Graphics/graphitem.cpp
  Graphics/palette.cpp
  Graphics/frameitem.cpp 
  Graphics/arrowitem.cpp
  Graphics/coordinatesystemitem.cpp
  Graphics/blipitem.cpp 
  Graphics/arena.cpp

  MVC/treeitem.cpp
  MVC/treemodel.cpp
  MVC/BlipModel/positionitem.cpp
  MVC/BlipModel/attitudeitem.cpp 
  MVC/BlipModel/velocityitem.cpp
  MVC/BlipModel/blipdebugitem.cpp
  MVC/BlipModel/blipcontainer.cpp
  MVC/BlipModel/blipmodel.cpp
)

set( mvs_core_GRAPHICS_HDRS
  Graphics/vertexitem.h
  Graphics/edgeitem.h
  Graphics/graphitem.h
  Graphics/graphgraphics.h
  Graphics/geometryitem.h
  Graphics/palette.h

  MVC/treeitem.h
)

if( WITH_GRAPHICS )
  list(APPEND mvs_core_SRCS ${mvs_core_GRAPHICS_SRCS} )
endif( WITH_GRAPHICS )


add_library(mvscore ${mvs_core_SRCS} )
if( WITH_GRAPHICS )
  qt5_use_modules(mvscore Core Gui Xml Network)
else (WITH_GRAPHICS )
  qt5_use_modules(mvscore Core Xml Network)
endif( WITH_GRAPHICS )

target_link_libraries( mvscore
  ${NDDS_LIBRARIES}
  ${AMG_LIBRARIES}
  ${VRONI_LIBRARIES}
  ${VISILIBITY_LIBRARIES}
  ${SDL_LIBRARY}
)

install(TARGETS mvscore
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${runtime_DIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${archive_DIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${library_DIR}
)

foreach(file ${mvs_core_HDRS} ${mvs_core_GRAPHICS_HDRS})
  string(REGEX MATCH "(.*)[/\\]" DIR ${file})
  install(FILES ${file} DESTINATION ${headers_DIR}/${DIR})
endforeach(file ${mvs_core_HDRS})

install(FILES   ${_codesynthesis_autogen_HDRS}
  DESTINATION ${headers_DIR}/XSD
)

install(FILES ${_rti_autogen_HDRS}
  DESTINATION ${headers_DIR}/IDL
)