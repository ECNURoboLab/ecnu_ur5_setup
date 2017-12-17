cmake_minimum_required(VERSION 2.8.3)
project(boost_numpy)

find_package(catkin REQUIRED)

# put our local cmake find scripts at the beginning of the cmake
# module search path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/libs/numpy/cmake ${CMAKE_MODULE_PATH})

# find required python packages
find_package(PythonInterp REQUIRED)
find_package(PythonLibsNew REQUIRED)
find_package(NumPy REQUIRED)

# find boost
# 
# set(Boost_USE_STATIC_LIBS ON)
# set(Boost_USE_MULTITHREADED ON) 
# set(Boost_USE_STATIC_RUNTIME ON)
FIND_PACKAGE(Boost COMPONENTS python REQUIRED)

message( STATUS "found boost:"
  "\nINCLUDE: ${Boost_INCLUDE_DIRS}"
  "\nLIB: ${Boost_LIBRARIES}"
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  LIBRARIES boost_numpy
#  DEPENDS system_lib
)

# add_definitions( -DBOOST_ALL_NO_LIB )
# In some cases, you may need to explicitly specify that a dynamic Boost is used; if so use:
add_definitions( -DBOOST_ALL_DYN_LINK )

# compiler definitions for non-windows builds
add_definitions(-fPIC)

# global settings for include paths
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${NUMPY_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

add_library(boost_numpy 
  # header files
  boost/numpy/dtype.hpp
  boost/numpy/internal.hpp
  boost/numpy/invoke_matching.hpp
  boost/numpy/matrix.hpp
  boost/numpy/ndarray.hpp
  boost/numpy/numpy_object_mgr_traits.hpp
  boost/numpy/scalars.hpp
  boost/numpy/ufunc.hpp

  # source files (in current directory)
  libs/numpy/src/dtype.cpp
  libs/numpy/src/scalars.cpp
  libs/numpy/src/ndarray.cpp
  libs/numpy/src/matrix.cpp
  libs/numpy/src/ufunc.cpp
  libs/numpy/src/numpy.cpp
)

target_link_libraries(boost_numpy
  ${Boost_LIBRARIES}
)

# install headers
install(DIRECTORY boost
    DESTINATION "include"
    FILES_MATCHING
    PATTERN "*.hpp"
    ${INSTALL_PERMISSIONS_SRC}
  )

# add submodules
#ADD_SUBDIRECTORY(libs/numpy/src)
#ADD_SUBDIRECTORY(libs/numpy/example)
#ADD_SUBDIRECTORY(libs/numpy/test)

