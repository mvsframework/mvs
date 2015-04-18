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
# Prepare my personal environment
# ===============================
#

# Load my macros 
include(cc_macros)

#
# 32/64 bit setup
# ===============
#
if( CMAKE_SIZEOF_VOID_P EQUAL 4 )
  # this is a 32bit machine
  set( HAVE_64_BIT OFF )
else()
  # this is a 64bit machine
  message(STATUS "CMake detected this machine to be 64bit.")
  set( HAVE_64_BIT ON )
endif()
mark_as_advanced( HAVE_64_BIT )

if(HAVE_64_BIT)
  option(BUILD_NATIVE_64_BIT "Do you want to build a native 64bit target?" ON)

  if( BUILD_NATIVE_64_BIT )
    message( STATUS "Configuring to build a native 64bit target.")
    set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
  else( BUILD_NATIVE_64_BIT )
    message( STATUS "Configuring to build a 32bit target on a 64bit system.")
    set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS OFF  )
    add_definitions(-m32)
    set( CMAKE_EXE_LINKER_FLAGS  "-m32")
    set( CMAKE_SHARED_LINKER_FLAGS  "-m32")
    set( CMAKE_MODULE_LINKER_FLAGS  "-m32")
    set( CMAKE_LINKER_FLAGS "-m32")
  #   message_variable(CMAKE_CXX_FLAGS)
  #   message_variable(CMAKE_EXE_LINKER_FLAGS)
  #   message_variable(CMAKE_SHARED_LINKER_FLAGS)
  #   message_variable(CMAKE_MODULE_LINKER_FLAGS)
  #   message_variable(LINK_FLAGS)
  endif( BUILD_NATIVE_64_BIT )
  
endif(HAVE_64_BIT)
#
# Prepare install target locations
# ================================
#

set(_cc_dest_BIN         "bin")
set(_cc_dest_LIB         "lib")
set(_cc_dest_ARCHIVE     "lib")
set(_cc_dest_INCLUDE     "include")

set(_cc_dest_BIN_64      "bin")
set(_cc_dest_LIB_64      "lib64")
set(_cc_dest_ARCHIVE_64  "lib64")
set(_cc_dest_INCLUDE_64  "include")

mark_as_advanced(
  _cc_dest_BIN
  _cc_dest_LIB
  _cc_dest_ARCHIVE
  _cc_dest_INCLUDE
  _cc_dest_BIN_64
  _cc_dest_LIB_64
  _cc_dest_ARCHIVE_64
  _cc_dest_INCLUDE_64
)



if( BUILD_NATIVE_64_BIT )
  set( runtime_DIR ${_cc_dest_BIN_64}     CACHE STRING "The binary installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( library_DIR ${_cc_dest_LIB_64}     CACHE STRING "The dynamic library installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( archive_DIR ${_cc_dest_ARCHIVE_64} CACHE STRING "The static library installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( headers_DIR ${_cc_dest_INCLUDE_64} CACHE STRING "The header installation location, relative to CMAKE_INSTALL_PREFIX.")
else( BUILD_NATIVE_64_BIT )
  set( runtime_DIR ${_cc_dest_BIN}     CACHE STRING "The binary installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( library_DIR ${_cc_dest_LIB}      CACHE STRING "The dynamic library installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( archive_DIR ${_cc_dest_ARCHIVE}  CACHE STRING "The static library installation location, relative to CMAKE_INSTALL_PREFIX.")
  set( headers_DIR ${_cc_dest_INCLUDE}  CACHE STRING "The header installation location, relative to CMAKE_INSTALL_PREFIX.")
endif( BUILD_NATIVE_64_BIT )








