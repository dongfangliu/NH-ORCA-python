
cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
add_subdirectory(RVO2/src)
include_directories(${RVO_SOURCE_DIR}/src)
add_subdirectory(pybind11)

set(pylib_name pyrvo2)


pybind11_add_module(${pylib_name} py_wrapper.cpp)
target_link_libraries(${pylib_name} PUBLIC RVO)

set_property(TARGET ${pylib_name}  PROPERTY CXX_STANDARD 14)
