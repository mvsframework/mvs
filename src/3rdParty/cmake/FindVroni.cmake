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
find_path(VRONI_INCLUDE_DIR vroni.h PATH_SUFFIXES vroni)

#debug output
if( NOT Vroni_FIND_QUIETLY )
  if( NOT VRONI_INCLUDE_DIR )
    message(SEND_ERROR ${VRONI_INCLUDE_DIR})
  else( NOT Vroni_FIND_QUIETLY )
    message(STATUS "Vroni include directory is " ${VRONI_INCLUDE_DIR} )
  endif( NOT VRONI_INCLUDE_DIR )
endif( NOT Vroni_FIND_QUIETLY )


#
# Find the actual library
# =======================
#

# Support preference of static libs by adjusting CMAKE_FIND_LIBRARY_SUFFIXES
if( Vroni_USE_STATIC_LIBS )
  set( _Vroni_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
  if(WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
  else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a )
  endif()
endif( Vroni_USE_STATIC_LIBS )

# do the finding
find_library(VRONI_Vroni_LIBRARY vroni)

# Restore the original find library ordering
if( Vroni_USE_STATIC_LIBS )
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${_Vroni_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
endif(Vroni_USE_STATIC_LIBS)




# debug output
if( NOT Vroni_FIND_QUIETLY )
  if( NOT VRONI_Vroni_LIBRARY )
    message(SEND_ERROR ${VRONI_Vroni_LIBRARY})
  else( NOT VRONI_Vroni_LIBRARY )
    message(STATUS "Vroni has been found as " ${VRONI_Vroni_LIBRARY} )
  endif( NOT VRONI_Vroni_LIBRARY )
endif( NOT Vroni_FIND_QUIETLY )

# find_path(VRONI_INCLUDE_DIR xxx.h)
# find_library(VRONI_xxx_LIBRARY xxx)
# find_library(VRONI_yyy_LIBRARY yyy)


#
# Prepare the standard return
# ===========================
#
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VRONI DEFAULT_MSG
  VRONI_INCLUDE_DIR VRONI_Vroni_LIBRARY)
if(VRONI_FOUND)
  set(VRONI_INCLUDE_DIRS ${VRONI_INCLUDE_DIR})
  set(VRONI_LIBRARIES ${VRONI_Vroni_LIBRARY} )
endif() 


# don't show the internal variables outside the advanced view
mark_as_advanced( 
  VRONI_INCLUDE_DIR
  VRONI_Vroni_LIBRARY
)