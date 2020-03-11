cmake_minimum_required(VERSION 2.8.3)
project(path_finding)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
) 

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OMPL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
  include/path_finding
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
    src/path_finding.cpp
    src/A_star.cpp
    src/JPS.cpp
    src/sample_base.cpp 
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OMPL_LIBRARIES}
)

