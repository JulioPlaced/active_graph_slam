# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(graph_d_exploration_CONFIG_INCLUDED)
  return()
endif()
set(graph_d_exploration_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(graph_d_exploration_SOURCE_PREFIX /home/julio/source/active_slam_project_github/src/d_opt_exploration)
  set(graph_d_exploration_DEVEL_PREFIX /home/julio/source/active_slam_project_github/devel/.private/graph_d_exploration)
  set(graph_d_exploration_INSTALL_PREFIX "")
  set(graph_d_exploration_PREFIX ${graph_d_exploration_DEVEL_PREFIX})
else()
  set(graph_d_exploration_SOURCE_PREFIX "")
  set(graph_d_exploration_DEVEL_PREFIX "")
  set(graph_d_exploration_INSTALL_PREFIX /home/julio/source/active_slam_project_github/install)
  set(graph_d_exploration_PREFIX ${graph_d_exploration_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'graph_d_exploration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(graph_d_exploration_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include " STREQUAL " ")
  set(graph_d_exploration_INCLUDE_DIRS "")
  set(_include_dirs "include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "None " STREQUAL " ")
    set(_report "Check the website 'None' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Julio Placed <jplaced@unizar.es>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${graph_d_exploration_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'graph_d_exploration' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'graph_d_exploration' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(graph_d_exploration_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND graph_d_exploration_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND graph_d_exploration_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT graph_d_exploration_NUM_DUMMY_TARGETS)
      set(graph_d_exploration_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::graph_d_exploration::wrapped-linker-option${graph_d_exploration_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR graph_d_exploration_NUM_DUMMY_TARGETS "${graph_d_exploration_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::graph_d_exploration::wrapped-linker-option${graph_d_exploration_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND graph_d_exploration_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND graph_d_exploration_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND graph_d_exploration_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/julio/source/active_slam_project_github/install/lib;/home/julio/source/active_slam_project_github/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(graph_d_exploration_LIBRARY_DIRS ${lib_path})
      list(APPEND graph_d_exploration_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'graph_d_exploration'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND graph_d_exploration_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(graph_d_exploration_EXPORTED_TARGETS "graph_d_exploration_generate_messages_cpp;graph_d_exploration_generate_messages_eus;graph_d_exploration_generate_messages_lisp;graph_d_exploration_generate_messages_nodejs;graph_d_exploration_generate_messages_py;graph_d_exploration_gencfg")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${graph_d_exploration_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 graph_d_exploration_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${graph_d_exploration_dep}_FOUND)
      find_package(${graph_d_exploration_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${graph_d_exploration_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(graph_d_exploration_INCLUDE_DIRS ${${graph_d_exploration_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(graph_d_exploration_LIBRARIES ${graph_d_exploration_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${graph_d_exploration_dep}_LIBRARIES})
  _list_append_deduplicate(graph_d_exploration_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(graph_d_exploration_LIBRARIES ${graph_d_exploration_LIBRARIES})

  _list_append_unique(graph_d_exploration_LIBRARY_DIRS ${${graph_d_exploration_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(graph_d_exploration_EXPORTED_TARGETS ${${graph_d_exploration_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "graph_d_exploration-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${graph_d_exploration_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
