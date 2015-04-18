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


set( CODESYNTHESIS_XSD_ARGS
  cxx-tree
  --namespace-map http://www.omg.org/IDL-Mapped/=XML::IDL
  --generate-polymorphic
  --type-naming ucc
  --function-naming lcc
  --root-element-last
)

set( CODESYNTHESIS_XSD_OUTPUT_SRC_EXTENSIONS
  .cxx
)

set( CODESYNTHESIS_XSD_OUTPUT_HDR_EXTENSIONS
  .hxx
)

file(MAKE_DIRECTORY ${CODESYNTHESIS_XSD_OUTPUT_DIR})

get_filename_component( _xsd_bin ${CODESYNTHESIS_XSD_FILE} NAME)

set( _codesynthesis_autogen_SRCS )
set( _codesynthesis_autogen_HDRS )

foreach( xsdfile ${XSD_SRCS} )
  get_filename_component( _file_name  ${xsdfile} NAME_WE)
  get_filename_component( _xsdfile_FQFN ${xsdfile} ABSOLUTE) # FQFN: fully-qualified-file-names

  message(STATUS "Preparing the processing of ${_xsdfile_FQFN}...")
  
  # Prepare some variables with the new names of the files resutling from processing
  # the original XSD sources.
  set( _codesynthesis_gen_src_files_FQFN )
  set( _codesynthesis_gen_hdr_files_FQFN )

  foreach( extension ${CODESYNTHESIS_XSD_OUTPUT_SRC_EXTENSIONS} )
    list(APPEND _codesynthesis_gen_src_files_FQFN ${CODESYNTHESIS_XSD_OUTPUT_DIR}/${_file_name}${extension} )
  endforeach( extension )
  foreach( extension ${CODESYNTHESIS_XSD_OUTPUT_HDR_EXTENSIONS} )
    list(APPEND _codesynthesis_gen_hdr_files_FQFN ${CODESYNTHESIS_XSD_OUTPUT_DIR}/${_file_name}${extension} )
  endforeach( extension )

  set( _codesynthesis_gen_files_FQFN 
    ${_codesynthesis_gen_src_files_FQFN}
    ${_codesynthesis_gen_hdr_files_FQFN}
  ) 
  message(STATUS "... resulting output files should be: " ${_codesynthesis_gen_files_FQFN})

  add_custom_command(OUTPUT ${_codesynthesis_gen_files_FQFN}
    # run CodeSynthesis on the XSD file, generate C++ files
    COMMAND ${CODESYNTHESIS_XSD_FILE}
    ARGS ${CODESYNTHESIS_XSD_ARGS} ${_xsdfile_FQFN}

    # copy the XSD files into the CODESYNTHESIS_XSD_TARGET_DIR ...
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy  ${_xsdfile_FQFN} ${CODESYNTHESIS_XSD_TARGET_DIR}
     

    DEPENDS ${_xsdfile_FQFN}
    WORKING_DIRECTORY ${CODESYNTHESIS_XSD_OUTPUT_DIR}
    COMMENT "calling :${CODESYNTHESIS_XSD_OUTPUT_DIR}/${_xsd_bin} ${CODESYNTHESIS_XSD_ARGS} ${_xsdfile_FQFN}"
  )

  # add the generated files to the sources for later compiling  If
  list(APPEND _codesynthesis_autogen_SRCS ${_codesynthesis_gen_src_files_FQFN} )
  list(APPEND _codesynthesis_autogen_HDRS ${_codesynthesis_gen_hdr_files_FQFN} )

endforeach( xsdfile )