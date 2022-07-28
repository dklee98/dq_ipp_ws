cmake_minimum_required(VERSION 3.0.2)
project(dq_ipp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  pcl_conversions
  pcl_ros
  voxblox_ros
  voxblox
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(rgbd
            src/rgbd.cpp
)

add_library(pn_ros
            src/planner_ros.cpp
)
add_library(pn
            src/planner.cpp
)
add_library(vb
            src/voxblox.cpp
)
add_library(rc
            src/ray_caster.cpp
)


catkin_package()

add_executable(rgbd_node src/rgbd_node.cpp)
target_link_libraries(rgbd_node rgbd ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} )

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node pn_ros pn vb rc ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} )
