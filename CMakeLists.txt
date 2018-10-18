# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set( CMAKE_CXX_STANDARD 11)

# The name of the project
PROJECT(RobMand2)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_BUILD_TYPE Release)

SET(RW_ROOT $ENV{RW_ROOT})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Add target executeable:
add_executable(Source pathplanning.cpp)

#Link all libraries and classes
target_link_libraries(Source ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

