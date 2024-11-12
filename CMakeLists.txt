cmake_minimum_required(VERSION 3.8)
project(sim_ros_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
    # fix C1128: number of sections exceeded object file format limit
    add_compile_options(/bigobj)
endif()

if(WIN32)
    # fix LNK1104: cannot open file 'libboost_regex-vc142-mt-x64-1_74.lib'
    set(Boost_USE_STATIC_LIBS ON)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(NOT DEFINED ENV{NAAD_WS_DIR})
  message(FATAL_ERROR "Environment variable NAAD_WS_DIR is not set")
endif()

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.1.7 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Boost REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros_manager)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros_manager)

coppeliasim_add_plugin(
    simROSManager
    LEGACY
    SOURCES
    src/sim_ros_manager.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionData.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionDataItem.cpp
)
coppeliasim_add_lua(lua/simROSManager.lua)
coppeliasim_add_lua(lua/simROSManager-ce.lua)

#add_library(simROSManager SHARED src/sim_ros_manager.cpp)
target_include_directories(simROSManager PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(simROSManager PRIVATE Boost::boost)

ament_target_dependencies(
    simROSManager
    PUBLIC
    rclcpp
    std_msgs
    std_srvs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simROSManager PRIVATE "SIM_ROS_MANAGER_BUILDING_LIBRARY")

add_executable(stop_sim src/stop_sim.cpp)
ament_target_dependencies(stop_sim rclcpp std_srvs)
install(TARGETS stop_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()