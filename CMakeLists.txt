cmake_minimum_required(VERSION 3.5.1)
PROJECT (HandSee)

if(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") OR (("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel") AND UNIX))
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++14 ")
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
  set(CMAKE_CXX_FLAGS "-std=c++14 ")
endif()

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED)

set(SRC
    src/main.cpp
    src/server.cpp
)

set(HEADERS
    include/server.hpp
    include/imgFeatures.hpp
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable( HandSee ${SRC} ${HEADERS})

target_link_libraries( HandSee ${OpenCV_LIBS})
