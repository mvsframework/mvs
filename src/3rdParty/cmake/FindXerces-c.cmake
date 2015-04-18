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
find_path(XERCESC_INCLUDE_DIR xercesc/parsers/SAXParser.hpp)

#debug output
if( NOT Xerces-c_FIND_QUIETLY )
  if( NOT XERCESC_INCLUDE_DIR )
    message(SEND_ERROR ${XERCESC_INCLUDE_DIR})
  else( NOT Xerces-c_FIND_QUIETLY )
    message(STATUS "Xerces-c include directory is " ${XERCESC_INCLUDE_DIR} )
  endif( NOT XERCESC_INCLUDE_DIR )
endif( NOT Xerces-c_FIND_QUIETLY )


#
# Find the actual library
# =======================
#
find_library(XERCESC_Xerces-c_LIBRARY xerces-c)

# debug output
if( NOT Xerces-c_FIND_QUIETLY )
  if( NOT XERCESC_Xerces-c_LIBRARY )
    message(SEND_ERROR ${XERCESC_Xerces-c_LIBRARY})
  else( NOT XERCESC_Xerces-c_LIBRARY )
    message(STATUS "Xerces-c has been found as " ${XERCESC_Xerces-c_LIBRARY} )
  endif( NOT XERCESC_Xerces-c_LIBRARY )
endif( NOT Xerces-c_FIND_QUIETLY )



#
# Prepare the standart return
# ===========================
#
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(XERCESC DEFAULT_MSG
  XERCESC_INCLUDE_DIR XERCESC_Xerces-c_LIBRARY)
if(XERCESC_FOUND)
  set(XERCESC_INCLUDE_DIRS ${XERCESC_INCLUDE_DIR})
  set(XERCESC_LIBRARIES ${XERCESC_Xerces-c_LIBRARY} )
endif() 


# don't show the internal variables outside the advanced view
mark_as_advanced( 
  XERCESC_INCLUDE_DIR
  XERCESC_Xerces-c_LIBRARY
)