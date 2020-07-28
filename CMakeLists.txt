CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(BST_INSTALL_CATEGORY Libraries) 

PROJECT(Gras)

###############################################################################
# Create library from Gras source files
###############################################################################
SET(SRC src/AStar.cpp src/SearchAlgorithm.cpp src/SearchNode.cpp)

IF (UNIX)
  SET_PROPERTY(SOURCE ${SRC} PROPERTY COMPILE_FLAGS "-O2")
ENDIF()

ADD_LIBRARY(Gras SHARED ${SRC})
TARGET_INCLUDE_DIRECTORIES(Gras PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

###############################################################################
# Create 2d grid search example with native console (no dependencies)
###############################################################################
ADD_EXECUTABLE(ExampleConsole examples/ExampleConsole.cpp)
TARGET_INCLUDE_DIRECTORIES(ExampleConsole PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples)
TARGET_LINK_LIBRARIES(ExampleConsole Gras)

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
  
ENDIF(Qt5_FOUND)

###############################################################################
# Create monkey search example
###############################################################################
FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osg osgViewer QUIET)

IF (OpenSceneGraph_FOUND)
  ADD_EXECUTABLE(ExampleMonkey examples/ExampleMonkey.cpp)
  TARGET_INCLUDE_DIRECTORIES(ExampleMonkey PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples ${OPENSCENEGRAPH_INCLUDE_DIRS})
  TARGET_LINK_LIBRARIES(ExampleMonkey Gras ${OPENSCENEGRAPH_LIBRARIES})
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
