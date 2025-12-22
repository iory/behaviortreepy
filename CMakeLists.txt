cmake_minimum_required(VERSION 3.16)
project(pybehaviortree LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Python and pybind11
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# BehaviorTree.CPP options - build as static library
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE ON CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)

add_subdirectory(BehaviorTree.CPP EXCLUDE_FROM_ALL)

# Find ZeroMQ for Groot2 support (after adding BehaviorTree.CPP for FindZeroMQ.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/BehaviorTree.CPP/cmake")
find_package(ZeroMQ QUIET)

# Create the Python module
pybind11_add_module(_behaviortreepy MODULE src/behavior_tree_wrapper.cpp)

target_include_directories(_behaviortreepy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BehaviorTree.CPP/include
)

# Add ZMQ_FOUND definition if ZeroMQ is available
if(ZeroMQ_FOUND)
    target_compile_definitions(_behaviortreepy PRIVATE ZMQ_FOUND)
    message(STATUS "ZeroMQ found - Groot2Publisher will be available")
else()
    message(STATUS "ZeroMQ NOT found - Groot2Publisher will NOT be available")
endif()

# Link with BehaviorTree.CPP library
# The library target is named 'behaviortree_cpp' (not 'behaviortree_cpp_v4')
target_link_libraries(_behaviortreepy PRIVATE behaviortree_cpp)

# Install the module
install(TARGETS _behaviortreepy LIBRARY DESTINATION behaviortreepy)
