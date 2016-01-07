cmake_minimum_required(VERSION 3.3)
project(SeniorProject)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    robot_odometry/include/Movingavg.h
    robot_odometry/scripts/trajectoryplotter_node.py
    robot_odometry/src/circular_odo_publisher.cpp
    robot_odometry/src/imu_orientation_node.cpp
    robot_odometry/src/Movingavg.cpp
    robot_odometry/src/robot_odometry_node.cpp
    robot_odometry/src/sandbox_node.cpp
    robot_odometry/src/vectornav_driver_node.cpp
    robot_odometry/srv/BiasImu.srv
    robot_odometry/srv/Uichangeaxis.srv
    robot_odometry/srv/Uiclear.srv
    robot_odometry/srv/Uisetstartpoint.srv
    robot_odometry/CMakeLists.txt
    robot_odometry/outodom.txt
    robot_odometry/package.xml
    visual_odometry/include/VisualOdometry.h
    visual_odometry/launch/open_webcam.launch
    visual_odometry/launch/visual_odometry.launch
    visual_odometry/msg/DistanceCovStamped.msg
    visual_odometry/scripts/video_publisher_node.py
    visual_odometry/src/camera_config_node.cpp
    visual_odometry/src/visual_odometry_node.cpp
    visual_odometry/src/VisualOdometry.cpp
    visual_odometry/CMakeLists.txt
    visual_odometry/outcov.txt
    visual_odometry/package.xml
    bug_moving_to_indigo.txt
    README.md)

add_executable(SeniorProject ${SOURCE_FILES})


include_directories(
        include ${catkin_INCLUDE_DIRS}
)