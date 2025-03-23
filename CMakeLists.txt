# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.12...3.29)
project(SimTraj)

find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)

# Copy the data directory to the build directory if dataset folder is not present in the build directory
#if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/dataset)
#  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../dataset DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#endif()

# file(COPY ../dataset DESTINATION .)

# create a target per cppfile
file(GLOB cppfiles ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
foreach(cppfile ${cppfiles})
  message("cppfile: ${cppfile}")
  get_filename_component(cppfile_name ${cppfile} NAME_WE)
  add_executable(${cppfile_name} ${cppfile})
  target_include_directories(${cppfile_name} PRIVATE ${GMP_INCLUDE_DIR})
  target_link_libraries(${cppfile_name} CGAL::CGAL ${GMP_LIBRARIES})
endforeach()
