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

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
SET(CMAKE_CXX_STANDARD 11)

# ----------------------------------------------------
# --- CXX FLAGS --------------------------------------
# ----------------------------------------------------

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-undefined")

# Project properties
SET(PROJECT_ORG thomascbrs)
SET(PROJECT_NAME walkgen)
SET(PROJECT_DESCRIPTION "Implementation of reactive footstepplanner with Mixed-integer for surface choices.")
SET(PROJECT_URL "https://github.com/thomascbrs/memmo_anymal")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(FOOTSTEP_PLANNER "Install foostep planner library" ON)
option(SURFACE_PLANNER "Install surface planner library" ON)
option(ROS_INTERFACE "Install the ros interface" ON)

# Check if the submodule cmake have been initialized
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
endif()

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_USE_CMAKE_EXPORT TRUE)

SETUP_PROJECT()
FINDPYTHON()
ADD_PROJECT_DEPENDENCY(pinocchio)

# Naming project libraries
SET(FOOTSTEP_PLANNER_NAME ${PROJECT_NAME}_footstep_planner)
SET(SURFACE_PLANNER_NAME ${PROJECT_NAME}_surface_planner)
SET(SURFACE_PROCESSING_NAME ${PROJECT_NAME}_surface_processing)
SET(ROS_INTERFACE_NAME ${PROJECT_NAME}_ros)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)

if(FOOTSTEP_PLANNER)
  ADD_SUBDIRECTORY("${FOOTSTEP_PLANNER_NAME}")
endif()

if(SURFACE_PLANNER)
  ADD_SUBDIRECTORY("${SURFACE_PLANNER_NAME}")
  ADD_SUBDIRECTORY("${SURFACE_PROCESSING_NAME}")
endif()

if(ROS_INTERFACE)
  ADD_SUBDIRECTORY("${ROS_INTERFACE_NAME}/python")
endif()
