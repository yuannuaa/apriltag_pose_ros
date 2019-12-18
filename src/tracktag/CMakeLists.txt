cmake_minimum_required(VERSION 2.8)
project(track_april_tag)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
    )


find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
endif()

find_package(apriltag REQUIRED)




include_directories(
${PROJECT_SOURCE_DIR}
${apriltag_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
/usr/local/include/apriltag
${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp 
    sensor_msgs 
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    nav_msgs
    )


add_library(${PROJECT_NAME} SHARED
${apriltag_LIBRARIES}
opencv_demo.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
/usr/local/lib/libapriltag.so
/usr/local/lib/libapriltag.so.3
/usr/local/lib/libapriltag.so.3.1.0
)

# Build examples


add_executable(april_tag_opencv
opencv_demo.cc)
target_link_libraries(april_tag_opencv 
${PROJECT_NAME}
${catkin_LIBRARIES})

add_executable(april_tag_opencv_emulate
opencv_emulate.cc)
target_link_libraries(april_tag_opencv_emulate 
${PROJECT_NAME}
${catkin_LIBRARIES})

