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
# Find the include directory
# =======================
#
find_path(AMG_INCLUDE_DIR amg.hpp PATH_SUFFIXES amg)

#debug output
if( NOT Amg_FIND_QUIETLY )
  if( NOT AMG_INCLUDE_DIR )
    message(SEND_ERROR ${AMG_INCLUDE_DIR})
  else( NOT Amg_FIND_QUIETLY )
    message(STATUS "AMG include directory is " ${AMG_INCLUDE_DIR} )
  endif( NOT AMG_INCLUDE_DIR )
endif( NOT Amg_FIND_QUIETLY )


#
# Find the actual library
# =======================
#

# Support preference of static libs by adjusting CMAKE_FIND_LIBRARY_SUFFIXES
if( Amg_USE_STATIC_LIBS )
  set( _Amg_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
  if(WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
  else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a )
  endif()
endif( Amg_USE_STATIC_LIBS )

# do the finding
find_library(AMG_Amg_LIBRARY amg)

# Restore the original find library ordering
if( Amg_USE_STATIC_LIBS )
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${_Amg_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
endif(Amg_USE_STATIC_LIBS)




# debug output
if( NOT Amg_FIND_QUIETLY )
  if( NOT AMG_Amg_LIBRARY )
    message(SEND_ERROR ${AMG_Amg_LIBRARY})
  else( NOT AMG_Amg_LIBRARY )
    message(STATUS "AMG has been found as " ${AMG_Amg_LIBRARY} )
  endif( NOT AMG_Amg_LIBRARY )
endif( NOT Amg_FIND_QUIETLY )

# find_path(AMG_INCLUDE_DIR xxx.h)
# find_library(AMG_xxx_LIBRARY xxx)
# find_library(AMG_yyy_LIBRARY yyy)


#
# Prepare the standart return
# ===========================
#
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AMG DEFAULT_MSG
  AMG_INCLUDE_DIR AMG_Amg_LIBRARY)
if(AMG_FOUND)
  set(AMG_INCLUDE_DIRS ${AMG_INCLUDE_DIR})
  set(AMG_LIBRARIES ${AMG_Amg_LIBRARY} )
endif() 


# don't show the internal variables outside the advanced view
mark_as_advanced( 
  AMG_INCLUDE_DIR
  AMG_Amg_LIBRARY
)