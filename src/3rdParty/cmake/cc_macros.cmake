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



############################################################
#
# MACRO: display a variable with its name and value
#
macro( message_variable VAR_NAME )
  list( LENGTH ${VAR_NAME} _listlength )
  if( _listlength LESS 2 )
    message( STATUS ${VAR_NAME} " = " ${${VAR_NAME}})
  else( _listlength LESS 2 )
    message( STATUS ${VAR_NAME} " = ")
    foreach( _entry ${${VAR_NAME}} )
      message( STATUS "|- " ${_entry})
    endforeach( _entry )
  endif( _listlength LESS 2 )
endmacro( message_variable VAR_NAME )
#
############################################################


############################################################
#
# MACRO: Create a set of messages to identify a CMakeLists file
#
macro( whoami )
  message("WHO AM I?\n"
          "     I AM " ${CMAKE_CURRENT_LIST_FILE} "\n"
          "       IN " ${CMAKE_CURRENT_LIST_DIR} )
endmacro( whoami )
#
############################################################


############################################################
#
# MACRO: build a static and a dynamic library from sources
#
macro( add_library_shared_and_static LIB_NAME LIB_SRCS )

  if( ${ARGC} GREATER 2 )
    message(STATUS "The macro ADD_LIBRARY_SHARED_AND_STATIC does not yet support ${ARGC} arguments." )
  else( ${ARGC} GREATER 2 )
  
    string(TOUPPER ${LIB_NAME} LIB_NAME_UPPER )
    
    set(LIB_${LIB_NAME_UPPER}         ${LIB_NAME}       )
    set(LIB_${LIB_NAME_UPPER}_STATIC  ${LIB_NAME}_STATIC)
    set(LIB_${LIB_NAME_UPPER}_TARGETS 
      ${LIB_${LIB_NAME_UPPER}} 
      ${LIB_${LIB_NAME_UPPER}_STATIC}
    )

    add_library( ${LIB_${LIB_NAME_UPPER}}        SHARED ${${LIB_SRCS}})
    add_library( ${LIB_${LIB_NAME_UPPER}_STATIC} STATIC ${${LIB_SRCS}})
    
    set_target_properties( ${LIB_${LIB_NAME_UPPER}}        PROPERTIES OUTPUT_NAME ${LIB_NAME})
    set_target_properties( ${LIB_${LIB_NAME_UPPER}_STATIC} PROPERTIES OUTPUT_NAME ${LIB_NAME})

    mark_as_advanced( 
      LIB_${LIB_NAME_UPPER}
      LIB_${LIB_NAME_UPPER}_STATIC
      LIB_${LIB_NAME_UPPER}_TARGETS
    )

    set( LIB_NAME_UPPER )
  
  endif(${ARGC} GREATER 2 ) 

endmacro( add_library_shared_and_static LIB_NAME LIB_SRCS )
#
############################################################





############################################################
#
# FUNCTION: process a subdirectory with sources
#
function(incorporate_subdir_src_tree 
  LOCAL_SRCS  # ARGV0
#   ${SUB_DIRS}  # (optional) ARGN
)

set(_current_DIR ${current_DIR})

# foreach( _src ${${LOCAL_SRCS}})
#   message(STATUS "${LOCAL_SRCS}   currently contains  :  ${_src}")
# endforeach()



foreach( subdir ${ARGN})
  
  set( current_DIR ${_current_DIR}/${subdir})
  message(STATUS "Adding to the INCLUDE directories: " ${current_DIR}  )
  include_directories(${current_DIR})
  
# Trigger the recursion...
#  message(STATUS "Loading subdir ${current_DIR}")
  include(${current_DIR}/CMakeLists.txt)

  # This _should_ also make the variable ${subdir}_SRCS available
  if(DEFINED ${subdir}_SRCS)
#     message("${subdir}_SRCS is defined. Importing it...")
#     foreach( _src ${${LOCAL_SRCS}})
#       message(STATUS "${LOCAL_SRCS}   currently contains  :  ${_src}")
#     endforeach()


    # processed the pushed up sources and prepend their directory name
    set(_subdir_SRCS) # reset the variable in each loop
    foreach( _src ${${subdir}_SRCS} )
#       message(STATUS "${LOCAL_SRCS} <-- ${subdir}/${_src}")
      set(_subdir_SRCS ${_subdir_SRCS} ${subdir}/${_src} )
    endforeach(_src)
    set( _src )
    
#     foreach( _src ${${LOCAL_SRCS}})
#       message(STATUS "${LOCAL_SRCS}   currently contains  :  ${_src}")
#     endforeach()
    
    # include the pushed up sources in the local sources
#     message("-adding-")
    set( ${LOCAL_SRCS}  ${${LOCAL_SRCS}} ${_subdir_SRCS} )

#     foreach( _src ${${LOCAL_SRCS}})
#       message(STATUS "${LOCAL_SRCS}   currently contains  :  ${_src}")
#     endforeach()
    
  else(DEFINED ${subdir}_SRCS)
    message(FATAL_ERROR "${current_DIR}/${subdir}/CMakeLists.txt did not provide ${subdir}_SRCS.") 
  endif(DEFINED ${subdir}_SRCS)

endforeach()
set( subdir )

# Push the sources into the parent's scope
# message(STATUS "Pushing  ${LOCAL_SRCS} into it's parent")
set( ${LOCAL_SRCS} ${${LOCAL_SRCS}} PARENT_SCOPE)
mark_as_advanced(${LOCAL_SRCS})

# message(STATUS "leaving")
endfunction(incorporate_subdir_src_tree)
#
############################################################