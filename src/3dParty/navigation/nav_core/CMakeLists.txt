cmake_minimum_required(VERSION 3.0.2)
project(nav_core)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            geometry_msgs
            tf2_ros
            costmap_2d
        )

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
            std_msgs
            geometry_msgs
            tf2_ros
            costmap_2d
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
