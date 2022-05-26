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

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------

# ------ SL1M --------------------
find_package(sl1m REQUIRED)
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import sl1m"
    RESULT_VARIABLE EXIT_CODE
    OUTPUT_QUIET
)

if (NOT ${EXIT_CODE} EQUAL 0)
  message(
          FATAL_ERROR
          "\n SL1M package is not installed. Turn off SURFACE_PLANNER option or install SL1M using : \"https://github.com/loco-3d/sl1m\"."
  )
endif()

SET(${SURFACE_PLANNER_NAME}_SOURCES
  __init__.py
  SurfacePlanner.py
  params.py
  )

SET(${SURFACE_PLANNER_NAME}_TOOLS_SOURCES
  __init__.py
  Surface.py
  collisions_utils.py
  )

SET(${SURFACE_PLANNER_NAME}_STANDALONE_SCENARIOS_SOURCES
  __init__.py
  anymal_stairs_planeseg.py
  anymal_stairs.py
  )

SET(${SURFACE_PLANNER_NAME}_TESTS_SOURCES
  __init__.py
  surface_planner_urdf.py
  surface_planner_planeseg.py
  )

FOREACH(src ${${SURFACE_PLANNER_NAME}_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PLANNER_NAME} ${src})
ENDFOREACH(src ${SURFACE_PLANNER_NAME}_SOURCES)

FOREACH(src ${${SURFACE_PLANNER_NAME}_TOOLS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PLANNER_NAME}/tools ${src})
ENDFOREACH(src ${SURFACE_PLANNER_NAME}_TOOLS_SOURCES)

FOREACH(src ${${SURFACE_PLANNER_NAME}_STANDALONE_SCENARIOS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PLANNER_NAME}/stand_alone_scenarios ${src})
ENDFOREACH(src ${SURFACE_PLANNER_NAME}_STANDALONE_SCENARIOS_SOURCES)

FOREACH(src ${${SURFACE_PLANNER_NAME}_TESTS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${SURFACE_PLANNER_NAME}/tests ${src})
ENDFOREACH(src ${SURFACE_PLANNER_NAME}_TESTS_SOURCES)

ADD_LIBRARY(${SURFACE_PLANNER_NAME} INTERFACE)
INSTALL(TARGETS ${SURFACE_PLANNER_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

INSTALL(DIRECTORY ${SURFACE_PLANNER_NAME}/data DESTINATION ${PYTHON_SITELIB}/${SURFACE_PLANNER_NAME})
