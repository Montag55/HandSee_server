cmake_minimum_required(VERSION 3.5.1)
PROJECT (HandSee)

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED)

set(SRC
    src/main.cpp    
)

set(HEADERS       
    include/header.hpp
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable( HandSee ${SRC} ${HEADERS})

target_link_libraries( HandSee ${OpenCV_LIBS})