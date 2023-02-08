cmake_minimum_required(VERSION 3.0.2)
project(ros_follow_line)

find_package(catkin REQUIRED COMPONENTS
    rospy
    nav_msgs
    std_srvs
    geometry_msgs
    cv_bridge
    sensor_msgs
)

catkin_python_setup()

catkin_package()


# ============================================================================
# Hooks
catkin_add_env_hooks(resource_paths SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hooks)

# ============================================================================
# Resources
install(PROGRAMS
	scripts/game_logic_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models worlds launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
