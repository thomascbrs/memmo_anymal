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

cmake_minimum_required(VERSION 3.16)

# Set up project properties
set(PROJECT_NAME walkgen)
SET(PROJECT_DESCRIPTION "Implementation of reactive footstepplanner with Mixed-integer for surface choices.")
set(PROJECT_URL https://github.com/thomas_cbrs/memmo_anymal)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  include(FetchContent)
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

# Include important cmake modules
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/stubs.cmake")

# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message("University of Edinburgh")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_USE_TEMPLATE_CSS YES)

set(CXX_DISABLE_WERROR False)
set(PROJECT_USE_CMAKE_EXPORT True)

# Create different building options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(FOOTSTEP_PLANNER "Install foostep planner library" ON)
option(SURFACE_PLANNER "Install surface planner library" ON)
option(SURFACE_PROCESSING "Install surface processing library" ON)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

check_minimal_cxx_standard(14 ENFORCE)

# Add the different required and optional dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 2.8.0 REQUIRED PKG_CONFIG_REQUIRES
                         "eigenpy >= 2.8.0")
endif(BUILD_PYTHON_INTERFACE)
add_project_dependency(pinocchio 2.6.3 REQUIRED PKG_CONFIG_REQUIRES
                       "pinocchio >= 2.6.3")

# # Naming project libraries
SET(FOOTSTEP_PLANNER_NAME ${PROJECT_NAME}_footstep_planner)
SET(SURFACE_PLANNER_NAME ${PROJECT_NAME}_surface_planner)
SET(SURFACE_PROCESSING_NAME ${PROJECT_NAME}_surface_processing)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)

if(FOOTSTEP_PLANNER)
  ADD_SUBDIRECTORY("${FOOTSTEP_PLANNER_NAME}")
endif()

if(SURFACE_PLANNER)
  ADD_SUBDIRECTORY("${SURFACE_PLANNER_NAME}")
endif()

if(SURFACE_PROCESSING)
  ADD_SUBDIRECTORY("${SURFACE_PROCESSING_NAME}")
endif()
