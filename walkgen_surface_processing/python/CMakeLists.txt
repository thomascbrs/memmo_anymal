# Copyright 2022 University of Edinburgh
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of  nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


SET(${SURFACE_PROCESSING_NAME}_SOURCES
  __init__.py
  surface_processing.py
  surface_detector.py
  surface_loader.py
  params.py
)

SET(${SURFACE_PROCESSING_NAME}_TOOLS_SOURCES
  __init__.py
  plot_tools.py
  Tess2.py
  geometry_utils.py
  SurfaceData.py
  transforms.py
  )

SET(${SURFACE_PROCESSING_NAME}_EXAMPLES_SOURCES
__init__.py
process_planeseg.py
process_stl.py
process_stl_folder.py
)


FOREACH(src ${${SURFACE_PROCESSING_NAME}_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PROCESSING_NAME} ${src})
ENDFOREACH(src ${SURFACE_PROCESSING_NAME}_SOURCES)

FOREACH(src ${${SURFACE_PROCESSING_NAME}_TOOLS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PROCESSING_NAME}/tools ${src})
ENDFOREACH(src ${SURFACE_PROCESSING_NAME}_TOOLS_SOURCES)

FOREACH(src ${${SURFACE_PROCESSING_NAME}_EXAMPLES_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PROCESSING_NAME}/examples ${src})
ENDFOREACH(src ${SURFACE_PROCESSING_NAME}_EXAMPLES_SOURCES)

ADD_LIBRARY(${SURFACE_PROCESSING_NAME} INTERFACE)
INSTALL(TARGETS ${SURFACE_PROCESSING_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

INSTALL(DIRECTORY ${SURFACE_PROCESSING_NAME}/data DESTINATION ${PYTHON_SITELIB}/${SURFACE_PROCESSING_NAME})
