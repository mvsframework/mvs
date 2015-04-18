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

option(NDDS_CONVERT_TO_XSD "Convert IDL data to XSD?" OFF)
mark_as_advanced(NDDS_CONVERT_TO_XSD)

set(NDDS_RTIDDSGEN_ARGS
  -d ${NDDS_RTIDDSGEN_OUTPUT_DIR}
  -language C++
  -namespace
  -stringSize 255
  -sequenceSize 100
  -replace
)

if( NDDS_CONVERT_TO_XSD )
  list(APPEND ${NDDS_RTIDDSGEN_ARGS} "-convertToXsd")
endif( NDDS_CONVERT_TO_XSD )


set(NDDS_RTIDDSGEN_OUTPUT_SRC_EXTENSIONS
  .cxx
  Plugin.cxx
  Support.cxx
)
set(NDDS_RTIDDSGEN_OUTPUT_HDR_EXTENSIONS
  .h
  Plugin.h
  Support.h
)

get_filename_component( _rtiddsgen_bin ${NDDS_RTIDDSGEN_FILE} NAME)


file( MAKE_DIRECTORY ${NDDS_RTIDDSGEN_OUTPUT_DIR} )

set( _rti_autogen_SRCS )
set( _rti_autogen_HDRS )

foreach( idlfile ${IDL_SRCS})
  get_filename_component( _file_name  ${idlfile} NAME_WE)
  get_filename_component( _idlfile_FQFN ${idlfile} ABSOLUTE)  # FQFN: fully-qualified-file-names

  message(STATUS "Preparing the processing of  ${_idlfile_FQFN}...")

  # Prepare some variables with the new names of the files resutling from processing
  # the original IDL sources.
  set(_rti_gen_src_files_FQFN )
  set(_rti_gen_hdr_files_FQFN  )

  foreach( extension ${NDDS_RTIDDSGEN_OUTPUT_SRC_EXTENSIONS})
    list(APPEND _rti_gen_src_files_FQFN  ${NDDS_RTIDDSGEN_OUTPUT_DIR}/${_file_name}${extension} )
  endforeach(extension)
  foreach( extension ${NDDS_RTIDDSGEN_OUTPUT_HDR_EXTENSIONS})
    list(APPEND _rti_gen_hdr_files_FQFN  ${NDDS_RTIDDSGEN_OUTPUT_DIR}/${_file_name}${extension} )
  endforeach(extension)
  list(APPEND _rti_gen_files_FQFN ${NDDS_RTIDDSGEN_OUTPUT_DIR}/${_file_name}.xsd )
  set( _rti_gen_files_FQFN
    ${_rti_gen_src_files_FQFN}
    ${_rti_gen_hdr_files_FQFN}
  )
  message(STATUS "... resulting output files should be: " ${_rti_gen_files_FQFN})

  list(APPEND _rti_autogen_SRCS ${_rti_gen_src_files_FQFN})
  list(APPEND _rti_autogen_HDRS ${_rti_gen_hdr_files_FQFN})

  # run rtidesgen on the file
#   add_custom_command(TARGET RTIDDSGEN_IDL PRE_BUILD

if( NDDS_CONVERT_TO_XSD )
  add_custom_command(OUTPUT ${_rti_gen_files_FQFN}
     # run rtidesgen on the file, generate C++ and XSD files
    COMMAND ${NDDS_RTIDDSGEN_FILE}
    ARGS ${NDDS_RTIDDSGEN_ARGS} ${_idlfile_FQFN}   
    
    # copy the XSD files into NDDS_XSD_TARGET_DIR ...
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy  ${NDDS_RTIDDSGEN_OUTPUT_DIR}/${_file_name}.xsd ${NDDS_XSD_TARGET_DIR}
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy  "rti_dds_topic_types_common.xsd" ${NDDS_XSD_TARGET_DIR} #NOTE: I have no idea why this file isn't generated in the -d specified output directory...
    
    # ... and delete the originals
    COMMAND ${CMAKE_COMMAND}
    ARGS -E remove ${_file_name}.xsd 
    COMMAND ${CMAKE_COMMAND}
    ARGS -E remove "rti_dds_topic_types_common.xsd" 
    
    DEPENDS ${_idlfile_FQFN}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} # This variable is provided by the "incorporate_subdir_src_tree" function
    COMMENT "calling :${CMAKE_CURRENT_SOURCE_DIR}/${_rtiddsgen_bin} ${NDDS_RTIDDSGEN_ARGS} ${_idlfile_FQFN}"
  )
else( NDDS_CONVERT_TO_XSD )
  add_custom_command(OUTPUT ${_rti_gen_files_FQFN}
    # run rtidesgen on the file, generate C++ and XSD files
    COMMAND ${NDDS_RTIDDSGEN_FILE}
    ARGS ${NDDS_RTIDDSGEN_ARGS} ${_idlfile_FQFN}   
    
    DEPENDS ${_idlfile_FQFN}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} # This variable is provided by the "incorporate_subdir_src_tree" function
    COMMENT "calling :${CMAKE_CURRENT_SOURCE_DIR}/${_rtiddsgen_bin} ${NDDS_RTIDDSGEN_ARGS} ${_idlfile_FQFN}"
  )
endif( NDDS_CONVERT_TO_XSD )

#   move_xsd_file( ${_file_name}.xsd )

endforeach(idlfile)