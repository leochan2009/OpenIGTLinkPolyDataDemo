cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(OpenIGTLinkPolyDataDemo)

find_package(VTK 7.0 REQUIRED)
include(${VTK_USE_FILE})

find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})

add_executable (PolyDataReceiver PolyDataReceiver.cpp)
target_link_libraries (PolyDataReceiver OpenIGTLink ${VTK_LIBRARIES})

add_executable (PolyDataServer PolyDataServer.cpp)
target_link_libraries (PolyDataServer OpenIGTLink ${VTK_LIBRARIES})

