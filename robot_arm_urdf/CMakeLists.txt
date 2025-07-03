cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_urdf)

# Find the catkin packages and components needed for this project.
# Based on the guide, we need Gazebo, ros_control, and standard ROS messages.
find_package(catkin REQUIRED
  COMPONENTS
    # Standard ROS packages
    roscpp
    rospy
    std_msgs
    geometry_msgs

    # URDF/Xacro related
    urdf
    xacro

    # Gazebo simulation packages
    gazebo_ros
    gazebo_ros_control

    # ROS controller packages
    controller_manager
    joint_state_controller
    position_controllers
)

##============================================================================
## CATKIN PACKAGE
##============================================================================
# This is critical for declaring dependencies to other ROS packages.
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    gazebo_ros
    gazebo_ros_control
    controller_manager
    joint_state_controller
    position_controllers
)

##============================================================================
## INSTALLATION
##============================================================================
# This installs your package's directories (launch, urdf, etc.)
# so they can be found by roslaunch and other tools.
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)