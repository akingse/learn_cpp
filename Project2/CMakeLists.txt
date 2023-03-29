# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.
cmake_minimum_required(VERSION 3.1...3.20)
#创建项目
project(Draw_point_set)

find_package(CGAL REQUIRED COMPONENTS Core OPTIONAL_COMPONENTS Qt5)
if(CGAL_Qt5_FOUND)
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()
# create a target per cppfile
file(
  GLOB cppfiles
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(cppfile ${cppfiles})
  create_single_source_cgal_program("${cppfile}")
endforeach()

if(CGAL_Qt5_FOUND)
  target_link_libraries (view_cloud PUBLIC CGAL::CGAL_Qt5)
  target_link_libraries (view_cloud PUBLIC CGAL::CGAL_Basic_viewer)
else()
  message(
    STATUS
      "NOTICE: The example draw_polygon_set requires Qt and drawing will be disabled."
  )
endif()
