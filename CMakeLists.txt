# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.12...3.29)
project(SimTraj)

find_package(CGAL REQUIRED)
# Copy the data directory to the build directory if dataset folder is not present in the build directory
#if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/dataset)
#  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../dataset DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#endif()

# file(COPY ../dataset DESTINATION .)

# create a target per cppfile
file(
  GLOB cppfiles
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
foreach(cppfile ${cppfiles})
  create_single_source_cgal_program("${cppfile}")
endforeach()

