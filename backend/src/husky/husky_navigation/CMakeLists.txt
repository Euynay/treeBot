cmake_minimum_required(VERSION 3.0.2)
project(husky_navigation)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  
endif()

install(
  DIRECTORY config launch maps
  FILES husky_gazebo/launch/empty_world.launch
        husky_gazebo/launch/husky_playpen.launch
        husky_gazebo/launch/playpen.launch
        husky_gazebo/launch/realsense.launch
        husky_gazebo/launch/spawn_husky.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

