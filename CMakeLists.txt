cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME archsearch)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_compile_options(-std=c++11)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_LIGHTWEIGHT "build library without visualization etc" OFF)

if(NOT BUILD_LIGHTWEIGHT)
  # OpenSceneGraph
  find_package(osg REQUIRED)
  find_package(osgDB REQUIRED)
  find_package(osgGA REQUIRED)
  find_package(osgUtil REQUIRED)
  find_package(osgViewer REQUIRED)
  set(OSG_LIBRARIES osg osgDB osgGA osgText osgUtil osgViewer)
  include_directories(${OSG_INCLUDE_DIRS})

  # OpenRAVE
  find_package(OpenRAVE REQUIRED)
  include_directories(${OpenRAVE_INCLUDE_DIRS})
endif()

# Point Cloud Library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost and python
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
find_package(Boost COMPONENTS python timer REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# OMPL
#find_package(ompl REQUIRED)
#include_directories(${OMPL_INCLUDE_DIRS})

# SBPL
find_package(SBPL REQUIRED)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

# grid_map
#find_package(grid_map_core REQUIRED)
#include_directories(${grid_map_core_INCLUDE_DIRS})

# grid_map (bypass the need for catkin etc by compiling the grid_map_core files inside our library)
set(grid_map_core_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/grid_map/grid_map_core/" CACHE FILEPATH "Path to grid_map_core folder.")
if(EXISTS "${grid_map_core_PATH}")
  include_directories(${grid_map_core_PATH}/include)
  file(GLOB grid_map_core_FILES
    ${grid_map_core_PATH}/src/*.cpp
    ${grid_map_core_PATH}/src/iterators/*.cpp
  )
  include(${grid_map_core_PATH}/cmake/grid_map_core-extras.cmake)
else()
  message(SEND_ERROR "grid_map_core NOT found. Please set the variable grid_map_core_PATH correctly.")
  return()
endif()

# Recast Library
set(RECAST_DIR "3rdparty/recastnavigation/")
set(RECAST_INCLUDE_DIRS
  ${RECAST_DIR}/DebugUtils/Include
  ${RECAST_DIR}/Detour/Include
  ${RECAST_DIR}/DetourTileCache/Include
  ${RECAST_DIR}/Recast/Include
  recast
)
include_directories(${RECAST_INCLUDE_DIRS})
file(GLOB RecastSources
  ${RECAST_DIR}/DebugUtils/Source/*.cpp
  ${RECAST_DIR}/Detour/Source/*.cpp
  ${RECAST_DIR}/DetourTileCache/Source/*.cpp
  ${RECAST_DIR}/Recast/Source/*.cpp
  ${RECASTDEMO_DIR}/Source/*.cpp
  recast/recastapp/*.cpp
)

# Library
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/flp/)
set(MyLibraryFiles
  ArchitectureSearchPy.cpp
  ArchitectureSearch.cpp
  ArchitectureSearchAnymal.cpp
  ArchitectureSearchDummy.cpp
  ArchitectureSearchStarleth.cpp
  flp/ActionSpace.cpp
  flp/EnvironmentConversions.cpp
  flp/EnvironmentGridMap.cpp
  flp/EnvironmentPCL.cpp
  flp/EnvironmentRecast.cpp
  flp/PlannerSBPL.cpp
  flp/PlannerRecast.cpp
  flp/PlannerConnectReferencePath.cpp
  flp/RobotModelFixedPrimitives.cpp
  flp/StateTransitionCost.cpp
  flp/Viewer.cpp
  ${RecastSources}
  ${grid_map_core_FILES}
)
if(BUILD_LIGHTWEIGHT)
  add_definitions(-DFLPLIGHTWEIGHT=1)
  set(MyLibraries
    ${PYTHON_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SBPL_LIBRARIES}
    ${grid_map_core_LIBRARIES}
  )
else()
  set(MyLibraryFiles ${MyLibraryFiles}
    flp/FeasibilityCheckerCollisionRAVE.cpp
    flp/RobotModelRAVE.cpp
    flp/ViewerRAVE.cpp
    flp/ViewerTrajopt.cpp
    flp/osgviewer/osgviewer.cpp
  )
  set(MyLibraries
    ${PYTHON_LIBRARIES}
    ${OSG_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
    ${OpenRAVE_CORE_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${SBPL_LIBRARIES}
    ${grid_map_core_LIBRARIES}
  )
endif()

add_library(ArchitectureSearch SHARED ${MyLibraryFiles})
target_link_libraries(ArchitectureSearch ${MyLibraries})

add_executable(test-arch test-arch.cpp)
target_link_libraries(test-arch ArchitectureSearch)

add_executable(test-starleth test-starleth.cpp)
target_link_libraries(test-starleth ArchitectureSearch)

add_executable(test-models test-models.cpp)
target_link_libraries(test-models ArchitectureSearch)

add_executable(test-subspace-models test-subspace-models.cpp)
target_link_libraries(test-subspace-models ArchitectureSearch)
