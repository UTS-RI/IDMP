cmake_minimum_required(VERSION 2.8.3)
project(idmp_ros)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb -fopenmp -fopenacc")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  sensor_msgs
  std_srvs
  visualization_msgs
  image_transport
  image_geometry
  message_filters
  depth_image_proc
  roslib
  pcl_ros
  message_generation
  interactive_markers
  # ros_numpy
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS thread system program_options)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 EXACT REQUIRED)
find_package(PCL REQUIRED)

add_service_files(
  FILES
  GetDistanceGradient.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp rospy tf tf2 image_transport
        sensor_msgs  std_srvs  visualization_msgs
        image_geometry message_filters depth_image_proc roslib
        #  DEPENDS
#        BOOST
#        EIGEN3
#        OpenCV

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(idmp
        src/IDMP_main.cpp
        src/IDMP/gp.cpp
        src/IDMP/octree.cpp
        src/IDMP/IDMP.cpp
        )

add_executable(publishGT tools/eval/publishGT.cpp)

target_link_libraries(
        idmp
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_link_libraries(
        publishGT
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${OpenCV_LIBRARIES}
)


add_dependencies(
        idmp
        idmp_ros_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
        publishGT
        ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS tools/queryTool.py
                        tools/evalTool.py
                        tools/speedMove.py
                        tools/datasetTFPublisher.py
                        tools/eval/dumpField.py
                        tools/eval/runTests.py
                        tools/eval/evalTest.py
                        tools/calibPose.py
                        tools/robotProximityViz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)