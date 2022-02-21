################################################################################
#
#  Copyright (c) 2020, Honda Research Institute Europe GmbH.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(BST_INSTALL_CATEGORY Libraries) 


################################################################################
# Check if project is stand-alone or inside tree with dependencies
################################################################################
SET(STANDALONE_PROJECT OFF)
IF(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  PROJECT(Gras)
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
  MESSAGE(STATUS "Compiling Gras as stand-alone project")
  SET(STANDALONE_PROJECT ON)
  SET(LIBRARY_LINKING_TYPE "SHARED")
  FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
  IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET(CMAKE_INSTALL_PREFIX "$ENV{SIT}/Libraries/Gras/1.0" CACHE PATH "Default sand box directory" FORCE)
  ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  INCLUDE(CompileSettings)
  #SET(BUILD_SHARED_LIBS ON)
  #SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
ELSE()
  MESSAGE(STATUS "Compiling ${PROJECT_NAME} as in-tree project")
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Gras/cmake)
ENDIF()

################################################################################
# Setup exported target collection
################################################################################
SET(GRAS_EXPORT_LIBRARIES "" CACHE INTERNAL "")
# Create a macro for adding an export library
FUNCTION(GRAS_ADD_EXPORT_LIBRARY library)
  SET(GRAS_EXPORT_LIBRARIES ${GRAS_EXPORT_LIBRARIES} ${library} CACHE INTERNAL "")
ENDFUNCTION()

################################################################################
# Add the automatically determined parts of the RPATH which point to directories
# outside the build tree to the install RPATH. 
# Note: This only works with CMake >= 3.0. With 2.8, the library directories of
# the dependencies must be added to the LD_LIBRARY_PATH manually in order to
# use the installed executables. This has no effect on using the package in
# another CMake project.
################################################################################
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

################################################################################
# The RPATH to be used when installing, but only if it's not a system directory
# See CMake wiki:
# (https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling)
################################################################################
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/$ENV{MAKEFILE_PLATFORM}")
ENDIF("${isSystemDir}" STREQUAL "-1")

###############################################################################
# Create library from Gras source files
###############################################################################
SET(SRC src/AStar.cpp src/SearchAlgorithm.cpp src/SearchNode.cpp)

IF (UNIX)
  SET_PROPERTY(SOURCE ${SRC} PROPERTY COMPILE_FLAGS "-O2")
ENDIF()

ADD_LIBRARY(Gras SHARED ${SRC})

#TARGET_INCLUDE_DIRECTORIES(Gras PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

TARGET_INCLUDE_DIRECTORIES(Gras PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    
    # Header install location
    $<INSTALL_INTERFACE:include>)

###############################################################################
# Create 2d grid search example with native console (no dependencies)
###############################################################################
ADD_EXECUTABLE(ExampleConsole examples/ExampleConsole.cpp)
TARGET_INCLUDE_DIRECTORIES(ExampleConsole PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples)
TARGET_LINK_LIBRARIES(ExampleConsole Gras)

# Install the libraries
INSTALL(TARGETS Gras EXPORT GrasExport DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
INSTALL(TARGETS ExampleConsole RUNTIME DESTINATION bin/$ENV{MAKEFILE_PLATFORM} LIBRARY DESTINATION lib/$ENV{MAKEFILE_PLATFORM})

# Install the headers
INSTALL(
  DIRECTORY src/ 
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# Mark for export
GRAS_ADD_EXPORT_LIBRARY(Gras)

###############################################################################
# Create 2d grid search example with Qt Gui
###############################################################################
FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets QUIET)

IF (Qt5_FOUND)
  ADD_EXECUTABLE(ExampleGrid examples/ExampleGrid.cpp examples/GridWidget.cpp)
  SET_TARGET_PROPERTIES(ExampleGrid PROPERTIES AUTOMOC TRUE)
  TARGET_INCLUDE_DIRECTORIES(ExampleGrid PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples)
  TARGET_LINK_LIBRARIES(ExampleGrid Gras Qt5::Core Qt5::Gui Qt5::Widgets)
  IF (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    TARGET_COMPILE_OPTIONS(ExampleGrid PRIVATE "-std=c++11")
  ENDIF()
  INSTALL(TARGETS ExampleGrid RUNTIME DESTINATION bin/$ENV{MAKEFILE_PLATFORM} LIBRARY DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
  
ENDIF(Qt5_FOUND)

###############################################################################
# Create monkey search example
###############################################################################
FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osg osgViewer QUIET)

IF (OpenSceneGraph_FOUND)
  ADD_EXECUTABLE(ExampleMonkey examples/ExampleMonkey.cpp)
  TARGET_INCLUDE_DIRECTORIES(ExampleMonkey PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples ${OPENSCENEGRAPH_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(ExampleMonkey Gras ${OPENSCENEGRAPH_LIBRARIES})
  INSTALL(TARGETS ExampleMonkey RUNTIME DESTINATION bin/$ENV{MAKEFILE_PLATFORM} LIBRARY DESTINATION lib/$ENV{MAKEFILE_PLATFORM})
ENDIF(OpenSceneGraph_FOUND)

###############################################################################
# copy the .dll file to the same folder as the executable
###############################################################################
IF (WIN32)
  ADD_CUSTOM_COMMAND(
      TARGET Gras POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
      ${LIBRARY_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Gras.dll
      ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}/Gras.dll)
ENDIF()

INCLUDE(ExportGras)
