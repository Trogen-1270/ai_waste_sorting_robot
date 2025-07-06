cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_urdf)

# Find the catkin packages and components needed for this project.
find_package(catkin REQUIRED
  COMPONENTS
    # Standard ROS packages
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation # Added for services

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
## BUILD SERVICES
##============================================================================
# This section tells catkin to build our custom .srv file.
add_service_files(
  FILES
  GetTargetPose.srv
)

# This generates the necessary message headers for C++ and Python.
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
   message_runtime # Added for services
)

##============================================================================
## INSTALLATION
##============================================================================
# This installs your package's directories so they can be found by ROS.
# We add the new 'srv' and 'scripts' directories here.
foreach(dir config launch meshes urdf worlds scripts srv)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)