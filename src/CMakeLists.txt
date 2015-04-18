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


#
# Deal with provided "3rdParty"-libs
# ==================================
# The custom(ized) 3rd party utility libraries are provided in /src/3rdParty/lib
# and /src/3rdParty/lib64, resp. Give the linke a hint to allow for an easier 
# find.
if( BUILD_NATIVE_64_BIT )
  set( 3RD_PARTY_LIBS_DIR ${PROJECT_SOURCE_DIR}/src/3rdParty/lib64)
else( BUILD_NATIVE_64_BIT )
  set( 3RD_PARTY_LIBS_DIR ${PROJECT_SOURCE_DIR}/src/3rdParty/lib )
endif( BUILD_NATIVE_64_BIT ) 


#
# Deal with Boost
# ===============
# openSuse currently doesn't have a RPM for boost_1_52 hence a small manual 
# "hack". As boost is headers only, no libraries are used.
#
include_directories(${BASE_DIR_BOOST})

# Deal with Eigen
# ===============
# Eigen is a matrix library that is required for the current Vroni interface.
# Eigen is a headers-only library, hence no libraries are used. 
#
find_package( Eigen3 REQUIRED QUIET)
if( EIGEN3_FOUND )
#   message(STATUS "Eigen ${EIGEN3_VERSION} has been found at ${EIGEN3_INCLUDE_DIR}")
  include_directories( ${EIGEN3_INCLUDE_DIR} )
  message_variable(EIGEN3_INCLUDE_DIR)
endif( EIGEN3_FOUND )


#
# Deal with Qt5
# =============
# 


# Allow to find Qt5 from non-standart location
list(APPEND CMAKE_PREFIX_PATH ${BASE_DIR_QT5})
include_directories( ${BASE_DIR_QT5}/include )

# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Xml REQUIRED) #NOTE: Deprecated, please use QXmlStreamReader and QXmlStreamWriter (in QtCore) for new functionality.
find_package(Qt5Network REQUIRED)

if( WITH_GRAPHICS )
  find_package(Qt5Gui REQUIRED)
endif( WITH_GRAPHICS )



#
# Deal with RTI DDS
# =================
# 


# although this depends on the BUILD_NATIVE_64_BIT choice, it also
# depends on a lot of other things, hence this is a _user_setting!

if( BUILD_NATIVE_64_BIT )
  set( NDDS_LIBRARY_DIR 
    "${BASE_DIR_NDDS}/lib/x64Linux2.6gcc4.4.5/" # 64 bit
#     "${BASE_DIR_NDDS}/lib/i86Linux2.6gcc4.4.5/" # 32 bit
    CACHE FILEPATH "The full path to the RTI specific libraries."
  )
elseif( BUILD_NATIVE_64_BIT )
  set( NDDS_LIBRARY_DIR 
  #   "${BASE_DIR_NDDS}/lib/x64Linux2.6gcc4.4.5/" # 64 bit
    "${BASE_DIR_NDDS}/lib/i86Linux2.6gcc4.4.5/" # 32 bit
    CACHE FILEPATH "The full path to the RTI specific libraries."
  )
endif( BUILD_NATIVE_64_BIT )

set( NDDS_LICENSE_FILE
  "${PROJECT_SOURCE_DIR}/rti_license.dat"
#   CACHE FILEPATH "The RTI license file."
)

set( NDDS_INCLUDE_DIRS ${BASE_DIR_NDDS}/include ${BASE_DIR_NDDS}/include/ndds
#   CACHE FILEPATH "The full path to the RTI specific include directories."
)

link_directories( ${NDDS_LIBRARY_DIR})

message_variable(NDDS_INCLUDE_DIRS)
include_directories( ${NDDS_INCLUDE_DIRS})

message(STATUS "Copying RTI license to ${CMAKE_CURRENT_BINARY_DIR}.")
file(COPY ${NDDS_LICENSE_FILE} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(INSTALL ${NDDS_LICENSE_FILE} DESTINATION ${runtime_DIR})

if( BUILD_NATIVE_64_BIT )
  set( NDDS_CXX_FLAGS 
    "-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT" # 64 bit
#   "-DRTI_UNIX -DRTI_LINUX "            # 32 bit
    CACHE STRING "RTI DDS flags for CXX builds."
  )
else( BUILD_NATIVE_64_BIT )
  set( NDDS_CXX_FLAGS 
#     "-DRTI_UNIX -DRTI_LINUX -DRTI_64BIT" # 64 bit
    "-DRTI_UNIX -DRTI_LINUX "            # 32 bit
    CACHE STRING "RTI DDS flags for CXX builds."
  )
endif( BUILD_NATIVE_64_BIT )

list(APPEND CMAKE_CXX_FLAGS ${NDDS_CXX_FLAGS} ) 

set( NDDS_REQUIRED_SYS_LIBS
  dl
  nsl
  m
  pthread
  rt
)

set( NDDS_REQUIRED_NDDS_LIBS
  nddscppz
  nddscz
  nddscorez
)

if( WITH_RTI_MONITORING )
  list(APPEND NDDS_REQUIRED_NDDS_LIBS rtimonitoringz)
endif( WITH_RTI_MONITORING )

  


set( NDDS_LIBRARIES ${NDDS_REQUIRED_NDDS_LIBS} ${NDDS_REQUIRED_SYS_LIBS} 
  CACHE INTERNAL "List of libraries that RTI DDS needs to link against."
)





#
# Deal with Xerces-c
# ==================
# Xerces-c is needed when loading the XML files by means of the generated 
# sources from CodeSynthesis XSD
#
find_package(Xerces-c REQUIRED QUIET)
if( XERCESC_FOUND )
  include_directories(${XERCESC_INCLUDE_DIRS})
  message_variable(XERCESC_INCLUDE_DIRS)
endif( XERCESC_FOUND )



#
# Deal with AMG
# ===============
# AMG is my own analytical mechanics and geometry/-graphy library
#
set(Amg_USE_STATIC_LIBS ON)
# find_package(Amg REQUIRED QUIET)
find_package(Amg REQUIRED QUIET)
if( AMG_FOUND )
  include_directories( ${AMG_INCLUDE_DIRS} )
  message_variable(AMG_INCLUDE_DIRS)
  message_variable(AMG_LIBRARIES)
endif(AMG_FOUND )


#
# Deal with VRONI
# ===============
# This block prepares the intergration of Vroni as a library that is already
# precompiled. Just make sure that the CMAKE_MODULE_PATH is set so that it can 
# be found.
#
set(Vroni_USE_STATIC_LIBS ON)
find_package(Vroni REQUIRED QUIET)
if( VRONI_FOUND )
  include_directories( ${VRONI_INCLUDE_DIRS} )
  message_variable(VRONI_INCLUDE_DIRS)
  message_variable(VRONI_LIBRARIES)
endif(VRONI_FOUND )


#
# Deal with VisiLibity
# ====================
# This block prepares the intergration of Vroni as a library that is already
# precompiled. Just make sure that the CMAKE_MODULE_PATH is set so that it can 
# be found.
#
set(VisiLibity_USE_STATIC_LIBS ON)
find_package(VisiLibity REQUIRED QUIET)
if( VISILIBITY_FOUND )
  include_directories( ${VISILIBITY_INCLUDE_DIRS} )
  message_variable( VISILIBITY_INCLUDE_DIRS )
  message_variable( VISILIBITY_LIBRARIES )
endif(VISILIBITY_FOUND )

# add_custom_command(TARGET mvscore POST_BUILD
add_custom_target(Doxygen
  COMMAND doxygen doxy.config
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/dox
  COMMENT "Running Doxygen:"
)

#
# Deal with SDL
# =============
# Simple Direct Media Layer (SDL, www.libsdl.org) is a small library that I use 
# for joystick access.
#
find_package(SDL REQUIRED QUIET)
if( SDL_FOUND )
  include_directories( ${SDL_INCLUDE_DIR} )
  message_variable(SDL_INCLUDE_DIR)
  message_variable(SDL_LIBRARY)
endif( SDL_FOUND )



#
# Continue with the subdirectories...
# ===================================
#


if( BUILD_MVS_CORE )
  add_subdirectory(core)
endif( BUILD_MVS_CORE )

add_subdirectory(periphery)
