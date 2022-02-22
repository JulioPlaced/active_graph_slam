cmake_minimum_required(VERSION 3.0.2)
project(move_slow_and_clear)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        nav_core
        pluginlib
        roscpp
        )


find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES move_slow_and_clear
    CATKIN_DEPENDS
        geometry_msgs
        nav_core
        pluginlib
        roscpp
)

add_library(${PROJECT_NAME} src/move_slow_and_clear.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)

install(TARGETS move_slow_and_clear
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


