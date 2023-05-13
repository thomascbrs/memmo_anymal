#!/usr/bin/env python3
#
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

ADD_SUBDIRECTORY("python")

# -------------------------------
# ---------cpp bindings----------
# -------------------------------

# FINDPYTHON()
# SEARCH_FOR_BOOST_PYTHON()
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED)
ADD_PROJECT_DEPENDENCY(eigenpy REQUIRED)

string(REGEX REPLACE "-" "_" PY_NAME ${SURFACE_PROCESSING_NAME})
set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})

# Main cpp Library
set(${SURFACE_PROCESSING_NAME}_HEADERS
  include/Types.hpp
  include/AffordanceLoader.hpp
)

set(${SURFACE_PROCESSING_NAME}_SOURCES_CPP
  src/AffordanceLoader.cpp
)

include_directories("${CMAKE_SOURCE_DIR}/${SURFACE_PROCESSING_NAME}/include")
ADD_LIBRARY(${SURFACE_PROCESSING_NAME}_cpp SHARED ${${SURFACE_PROCESSING_NAME}_SOURCES_CPP} ${${SURFACE_PROCESSING_NAME}_HEADERS})

# Include Eigen3 directories
TARGET_INCLUDE_DIRECTORIES(${SURFACE_PROCESSING_NAME}_cpp SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${SURFACE_PROCESSING_NAME}_cpp SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})

# Find pinocchio library
# ADD_PROJECT_DEPENDENCY(pinocchio REQUIRED)
# TARGET_LINK_LIBRARIES(${SURFACE_PROCESSING_NAME}_cpp PUBLIC pinocchio::pinocchio)

# Find parametric curves library and headers
# ADD_PROJECT_DEPENDENCY(ndcurves REQUIRED)
# TARGET_LINK_LIBRARIES(${SURFACE_PROCESSING_NAME}_cpp PUBLIC ndcurves::ndcurves)

# Find eiquadprog library and headers
ADD_PROJECT_DEPENDENCY(eiquadprog REQUIRED)
TARGET_LINK_LIBRARIES(${SURFACE_PROCESSING_NAME}_cpp PUBLIC eiquadprog::eiquadprog)

ADD_PROJECT_DEPENDENCY(hpp-fcl REQUIRED)
TARGET_LINK_LIBRARIES(${SURFACE_PROCESSING_NAME}_cpp PUBLIC hpp-fcl::hpp-fcl)

ADD_PROJECT_DEPENDENCY(hpp-affordance REQUIRED)
TARGET_LINK_LIBRARIES(${SURFACE_PROCESSING_NAME}_cpp PUBLIC hpp-affordance::hpp-affordance)


# Add suffix version
if(SUFFIX_SO_VERSION)
  set_target_properties(${SURFACE_PROCESSING_NAME}_cpp PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

target_compile_options(${SURFACE_PROCESSING_NAME}_cpp PUBLIC -DNDEBUG -O3)

# Python Bindings
set(${PY_NAME}_BINDINGS_SOURCES
  bindings/vector-converter.hpp
  bindings/walkgen_processing.cpp
)
ADD_LIBRARY(${PY_NAME}_pywrap SHARED ${${PY_NAME}_BINDINGS_SOURCES})
TARGET_LINK_LIBRARIES(${PY_NAME}_pywrap PUBLIC ${SURFACE_PROCESSING_NAME}_cpp ${PYTHON_LIBRARY} eigenpy::eigenpy)
TARGET_INCLUDE_DIRECTORIES(${PY_NAME}_pywrap SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})
set_target_properties(${PY_NAME}_pywrap PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME})
TARGET_LINK_BOOST_PYTHON(${PY_NAME}_pywrap PUBLIC)
install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

# Installation
install(TARGETS ${SURFACE_PROCESSING_NAME}_cpp EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
