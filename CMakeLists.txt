cmake_minimum_required(VERSION 2.8.3)
project(joy_stick_drone_pkgs)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package()

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joydrone src/joystick.cpp)
target_link_libraries(joydrone ${catkin_LIBRARIES})
add_dependencies(joydrone joy_stick_drone_pkgs_generate_messages_cpp)
