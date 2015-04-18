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
# Deal with CMake 
# ===============
# Some "global" cmake settings to make this project configer correctly
#

#
# NOTE: the actual compiler used is set through environmental variables CXX and
# CC, see http://cmake.org/Wiki/CMake_Useful_Variables#Compilers_and_Tools
# kdevelop sets those as part of the "environment", configurable in the project
# settings.
#


project(mvs-core)
cmake_minimum_required(VERSION 2.8.9)


# load my CMake Stuff
get_filename_component(_cc_projects_DIR ${CMAKE_CURRENT_LIST_DIR}/../../svn/claus/Projects  ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/src/3rdParty/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/src/3rdParty")
include(cc_general) 

# install into the source dir of this project
set( CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} )

# enable C++11
add_definitions(-std=gnu++11)

# show all warnings during the compilation
add_definitions(-Wall)


#
# Configuration Options
# =====================
#
option( WITH_GRAPHICS
  "Should the executable have a graphical interface?"
  ON)

option( DEBUG_GRAPHICS
  "Should extra debugging graphics be generated and shown? 
  (NOTE: Will create visual clutter; Requires WITH_GRAPHICS)"
  OFF)

option( DEBUG_CONSOLE_COLORS
  "Should console (debug) output be colorized? 
  (NOTE: Might not work on all console implementations.)"
  OFF)

option( WITH_RFSIMULATION
  "Should the executable simulate R/F propagation to decide on COM connectedness?
   (NOTE: This option might not do what you think it does. At this point, leave it ON...)"
  ON)

option( WITH_MATLABENGINE
  "Should the Nodes include a MATLAB engine interface?"
  OFF)


option( WITH_RTI_MONITORING
  "Should RTI Connext link in the monitoring extensions?"
  OFF)

option( BUILD_MVS_CORE
  "Should the simulator core library be build from sources?"
  ON)

# as we need to keep track of the options in order to be able to properly 
# include the headers files, generate a header file that notes all the options
configure_file(
  ${PROJECT_SOURCE_DIR}/src/config.h.in
  ${PROJECT_BINARY_DIR}/config.h 
)
include_directories(${PROJECT_BINARY_DIR}) # so that config.h can be found

#
# Deal with Qt5
# =============
# 

# preapare Qt5 inclusion from non-standart location
set(BASE_DIR_QT5
#   "/home/burdell/Qt5/5.0.2/gcc_64" # pre-compiled 64 bit (release) libs
#   "/home/claus/Qt/qt-everywhere-opensource-src-5.2.0/qtbase/" # self-compiled (debug) libs 
#   "/home/claus/Qt/5.0.2-opensource-offline/5.0.2/gcc_64/" # pre-compiled (release) libs
  "~/Qt5/qt-opensource-linux-x64-5.3.2/5.3/gcc_64/" # pre-compiled (release) libs
  CACHE FILEPATH "The full path to the Qt 5 installation."
) 


#
# Deal with RTI DDS
# =================
# 

set( BASE_DIR_NDDS
#   "/home/burdell/RTI/ndds.5.0.0"
  "~/RTI/ndds.5.1.0"
  CACHE FILEPATH "The full path to the RTI Connext DDS installation."
)


#
# Deal with Boost
# ===============
# openSuse currently doesn't have a RPM for boost_1_52 hence a small manual 
# "hack". As boost is headers only, no libraries are used.
#
set( BASE_DIR_BOOST 
#   "/usr/include"                # system version
#   "~/git/modular-boost"         # trunk, git version
#   "/home/burdell/boost_1_56_0"  # specific, downloaded version
  "~/boost/boost_1_56_0" 
  CACHE FILEPATH "The full path to the Boost include directory."
)


#
# Deal with MATLAB
# ================
#
set(BASE_DIR_MATLAB
  "/usr/local/MATLAB/R2014a"
  CACHE FILEPATH "The full path to the MATLAB installation."
)


add_subdirectory(src)