cmake_minimum_required(VERSION 2.8.3)
project(hexa_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  Transfert.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime
)

if ("$ENV{RESIBOTS_DIR}" STREQUAL "")
  set(libdynamixel_dir "~")
else()
  set(libdynamixel_dir "$ENV{RESIBOTS_DIR}")
endif()

link_directories("${libdynamixel_dir}/lib")
include_directories(
  ${catkin_INCLUDE_DIRS}
  "${libdynamixel_dir}/include"
)

add_executable(hexapod_client src/hexapod_client.cpp)
target_link_libraries(hexapod_client ${catkin_LIBRARIES} dynamixel)

add_executable(hexapod_server src/hexapod_server.cpp src/robotHexa.cpp src/controllerPhase.cpp src/controllerDuty.cpp)
target_link_libraries(hexapod_server ${catkin_LIBRARIES} dynamixel)

add_executable(limit_torque_ax18 src/limit_torque_ax18.cpp)
target_link_libraries(limit_torque_ax18 ${catkin_LIBRARIES} dynamixel)

add_executable(save_traj src/save_traj.cpp)
target_link_libraries(save_traj ${catkin_LIBRARIES} dynamixel)
