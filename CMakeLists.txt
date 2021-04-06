cmake_minimum_required(VERSION 2.8.3)
project(verify_aruco)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_eigen
  reef_msgs
        message_filters
        tf2_ros
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include
        SYSTEM ${BOOST_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/verify_aruco_node.cpp)

add_executable(${PROJECT_NAME}_tf src/verify_aruco_tf.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

