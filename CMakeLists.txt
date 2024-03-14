cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -march=native -mno-avx -mno-avx2 -Wunused-function " ) #-fstack-protector -fsanitize=address -static-libasan ")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  rosbag_storage
  livox_ros_driver
  message_generation
  eigen_conversions
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
  deps/lidar_intensity_correction/include
)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

file(GLOB lidar_intensity_correction_srcs ${PROJECT_SOURCE_DIR}/deps/lidar_intensity_correction/src/*.cpp)

add_executable(fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd-Tree/ikd_Tree.cpp
  src/use-ikfom.cpp
  src/common_lib.cpp
  src/IMU_Processing.cpp
  src/preprocess.cpp
  src/reflectance_grad.cpp
)
add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(fastlio_mapping ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} )
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
target_include_directories(fastlio_mapping PRIVATE INTERFACE )

add_executable(compensate_intensity
  src/compensate_intensity_node.cpp
  src/input_pcl_filter.cpp
  ${lidar_intensity_correction_srcs}
)
add_dependencies(compensate_intensity ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_definitions(compensate_intensity PRIVATE COMP_ONLY)
target_link_libraries(compensate_intensity ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} )
target_include_directories(compensate_intensity PRIVATE ${PYTHON_INCLUDE_DIRS})
target_include_directories(compensate_intensity PRIVATE INTERFACE )
