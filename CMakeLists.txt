CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

# Project properties
SET(PROJECT_ORG memmo_anymal)
SET(PROJECT_NAME walkgen)
SET(PROJECT_DESCRIPTION "Implementation of reactive footstepplanner with Mixed-integer for surface choices.")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()


# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
include(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)

# Disable -Werror on Unix.
SET(CXX_DISABLE_WERROR True)
# Undefined references errors for debug purpose.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-undefined")


# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
set(BOOST_COMPONENTS unit_test_framework)
ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED)

if(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  ADD_PROJECT_DEPENDENCY(eigenpy REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
  set(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
endif()

SEARCH_FOR_BOOST_PYTHON()

# Main cpp Library
set(${PROJECT_NAME}_HEADERS
  include/FootTrajectoryBezier.hpp
)

set(${PROJECT_NAME}_SOURCES_CPP
  src/FootTrajectoryBezier.cpp
)

add_library(${PROJECT_NAME}_cpp SHARED ${${PROJECT_NAME}_SOURCES_CPP} ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME}_cpp PUBLIC $<INSTALL_INTERFACE:include>)

# Include Eigen3 directories
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_cpp SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_cpp SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})

# Find pinocchio library
add_project_dependency(pinocchio REQUIRED)

# Link pinocchio library
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_cpp PUBLIC pinocchio::pinocchio)

# Find parametric curves library and headers
add_project_dependency(ndcurves REQUIRED)

# Link parametric curves library
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC ndcurves::ndcurves)

# Find eiquadprog library and headers
add_project_dependency(eiquadprog REQUIRED)

# Link eiquadprog library
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC eiquadprog::eiquadprog)

add_project_dependency(yaml-cpp CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME}_cpp PUBLIC ${YAML_CPP_LIBRARIES})

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME}_cpp PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

target_compile_options(${PROJECT_NAME}_cpp PUBLIC -DNDEBUG -O3)


# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  set(${PY_NAME}_BINDINGS_SOURCES
    bindings/walkgen.cpp
  )
  add_library(${PY_NAME}_pywrap SHARED ${${PY_NAME}_BINDINGS_SOURCES})
  target_link_libraries(${PY_NAME}_pywrap PUBLIC ${PROJECT_NAME}_cpp ${PYTHON_LIBRARY} eigenpy::eigenpy)
  target_include_directories(${PY_NAME}_pywrap SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})
  set_target_properties(${PY_NAME}_pywrap PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME})
  TARGET_LINK_BOOST_PYTHON(${PY_NAME}_pywrap PUBLIC)
  install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

endif()

# Installation
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(TARGETS ${PROJECT_NAME}_cpp EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
  install(TARGETS ${PROJECT_NAME}_cpp RUNTIME DESTINATION bin)
endif()



SET(${PROJECT_NAME}_SOURCES
  __init__.py
  SurfacePlanner.py
  FootStepPlanner.py
  FootStepTrajectory.py
  GaitManager.py
  contact.py
  FootStepManager.py
  WalkgenRosMessageConversion.py
  params.py
  )

SET(${PROJECT_NAME}_STANDALONE_SCENARIOS_SOURCES
  __init__.py
  anymal_stairs.py
  )

SET(${PROJECT_NAME}_TOOLS_SOURCES
  __init__.py
  plot_tools.py
  SurfaceData.py
  Tess2.py
  geometry_utils.py
  heightmap.py
  heightmap_generator.py
  optimisation.py
  )

SET(${PROJECT_NAME}_TESTS_SOURCES
  __init__.py
  surface_planner.py
  )

FOREACH(src ${${PROJECT_NAME}_SOURCES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME} ${src})
ENDFOREACH(src ${PROJECT_NAME}_SOURCES)

FOREACH(src ${${PROJECT_NAME}_STANDALONE_SURFACES_SOURCES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME}/stand_alone_scenarios/surfaces ${src})
ENDFOREACH(src ${PROJECT_NAME}_STANDALONE_SURFACES_SOURCES)

FOREACH(src ${${PROJECT_NAME}_TESTS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME}/tests ${src})
ENDFOREACH(src ${PROJECT_NAME}_TESTS_SOURCES)

FOREACH(src ${${PROJECT_NAME}_TOOLS_SOURCES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME}/tools ${src})
ENDFOREACH(src ${PROJECT_NAME}_TOOLS_SOURCES)

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

INSTALL(DIRECTORY walkgen/data
  DESTINATION ${PYTHON_SITELIB}/walkgen)
