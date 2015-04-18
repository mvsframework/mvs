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
# Deal with mvs_core (Multi-UAV-Simulation Core)
# ==========================================
#
if( BUILD_MVS_CORE )
  message_variable(CMAKE_CURRENT_BINARY_DIR)
  message_variable(CMAKE_CURRENT_LIST_DIR)
  include_directories( ${CMAKE_CURRENT_BINARY_DIR}/../core )
  include_directories( ${CMAKE_CURRENT_LIST_DIR}/../core )
  set(MVS_CORE_LIBRARIES mvscore) # simply use the target in core
else( BUILD_MVS_CORE )
  include_directories( ${CMAKE_INSTALL_PREFIX}/${headers_DIR} )
  set(MVS_CORE_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${archive_DIR}/libmvscore.a)
endif( BUILD_MVS_CORE )



#
# Deal with Qt5
# =============
# 

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)



#
# Deal with MATLAB
# ================
# FindMatlab8.cmake uses "MATLAB_DIR" as it's base directory to start the its
# it's search. As I elected to call all base directories in a different name
# scheme, some conversion is necessary
#
set(MATLAB_LIBRARIES)
if( WITH_MATLABENGINE )
  set(MATLAB_DIR ${BASE_DIR_MATLAB} CACHE FILEPATH "The path used by FindMatlab8.cmake")
  message_variable( MATLAB_DIR )
  find_package( Matlab8 REQUIRED )
  if( MATLAB_FOUND )
    include_directories( ${MATLAB_INCLUDE_DIR} )
    message_variable( MATLAB_INCLUDE_DIR )
    message_variable( MATLAB_LIBRARIES )
  endif( MATLAB_FOUND )
endif( WITH_MATLABENGINE )


#
# The actual compilation work for PERIPHERY
# =========================================
#


set( nodes_SRCS
  uav.cpp
  launcher.cpp
  main.cpp
)
set( nodes_QT5MODULES Xml Network)

set(nodes_binary_name)
if( WITH_GRAPHICS )
  add_subdirectory(GUI)
  set(nodes_binary_name "gnodes")
else( WITH_GRAPHICS )
  set(nodes_binary_name "cnodes")
endif( WITH_GRAPHICS )

add_executable(${nodes_binary_name}
  ${nodes_SRCS}
)
qt5_use_modules(${nodes_binary_name} ${nodes_QT5MODULES})

target_link_libraries(${nodes_binary_name}
  ${MVS_CORE_LIBRARIES}
  ${NDDS_LIBRARIES}
  ${XERCESC_LIBRARIES} 
  ${AMG_LIBRARIES}
  ${VRONI_LIBRARIES}
  ${VISILIBITY_LIBRARIES}
  ${SDL_LIBRARY}
  ${MATLAB_LIBRARIES} # this is an optional lib, check 'WITH_MATLABENGINE'

)

install(TARGETS ${nodes_binary_name}
  RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/${runtime_DIR}
)
