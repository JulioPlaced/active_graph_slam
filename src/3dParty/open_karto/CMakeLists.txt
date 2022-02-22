cmake_minimum_required(VERSION 3.0.2)
project(open_karto)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS
    include
  LIBRARIES
    karto
)

if(BUILD_SHARED_LIBS)
  add_definitions(-DKARTO_DYNAMIC)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)
target_link_libraries(karto ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
