cmake_minimum_required(VERSION 3.0)
 
PROJECT(Simulation)

# Path to Eigen headers
include_directories( "K:/lib-x64-msvc2019/eigen" )

# Path to Virtuose haptic library
SET(CMAKE_MODULE_PATH ${Simulation_SOURCE_DIR} )
INCLUDE_DIRECTORIES(${Simulation_SOURCE_DIR} "K:/lib-x64-msvc2019/Virtuose/VirtuoseAPI_v3_97")
LINK_DIRECTORIES("K:/lib-x64-msvc2019/Virtuose/VirtuoseAPI_v3_97/win/lib/VC2017/x64/Release")

FIND_PACKAGE(H3DUtil REQUIRED)
INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 

FIND_PACKAGE(HAPI REQUIRED)
INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 

find_package(VTK REQUIRED)
if (NOT VTK_FOUND)
  message("Skipping Simulation: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()

message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  # old system
  include(${VTK_USE_FILE})
  add_executable(Simulation MACOSX_BUNDLE Energy.h Solver.h ParticleSystem.h Interactor.h Simulation.cpp )
  target_link_libraries(Simulation ${H3DUTIL_LIBRARIES} ${HAPI_LIBRARIES} ${VTK_LIBRARIES} virtuoseDLL.lib)
  # make the name of debug libraries end in _d.
  SET_TARGET_PROPERTIES( Simulation PROPERTIES DEBUG_POSTFIX "_d" )
 
else ()
  # include all components
  add_executable(Simulation MACOSX_BUNDLE Energy.h Solver.h ParticleSystem.h Interactor.h Simulation.cpp )
  target_link_libraries(Simulation ${H3DUTIL_LIBRARIES} ${HAPI_LIBRARIES} ${VTK_LIBRARIES} virtuoseDLL.lib)
  # make the name of debug libraries end in _d.
  SET_TARGET_PROPERTIES( Simulation PROPERTIES DEBUG_POSTFIX "_d" )
  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS Simulation
    MODULES ${VTK_LIBRARIES}
    )
endif () 
