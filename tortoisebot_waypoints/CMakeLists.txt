cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_waypoints)

# Link directories link_directories(/opt/homebrew/opt/box2d/lib /opt/homebrew/opt/glfw/lib)

find_package(catkin REQUIRED COMPONENTS
  rospy actionlib actionlib_msgs geometry_msgs nav_msgs std_msgs tf message_generation
)

add_action_files(FILES WaypointAction.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs geometry_msgs nav_msgs std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

# Install scripts so roslaunch/rostest can run them directly
catkin_install_python(PROGRAMS
  src/tortoisebot_action_server.py
  test/test_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/waypoints_test.test)
endif()
