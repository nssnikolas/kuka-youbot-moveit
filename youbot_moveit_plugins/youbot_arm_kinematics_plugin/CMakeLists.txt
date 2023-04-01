cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_kinematics_plugin)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_EXE_LINKER_FLAGS "--no-as-needed")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  eigen_conversions
  control_msgs
  pr2_mechanism_msgs
  pr2_controllers_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread)


catkin_package(
  LIBRARIES youbot_arm_kinematics_plugin
  CATKIN_DEPENDS moveit_core
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(youbot_arm_kinematics_plugin
	src/youbot_arm_kinematics_plugin.cpp
	src/arm_analytical_inverse_kinematics.cpp
	src/arm_kdl_forward_kinematics.cpp
	src/arm_kdl_inverse_kinematics.cpp
	src/inverse_kinematics_solver.cpp
	src/solver_info_processor.cpp
	src/pr2_arm_kinematics_utils.cpp
)

target_link_libraries(youbot_arm_kinematics_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS youbot_arm_kinematics_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES
   youbot_arm_kinematics_plugins.xml
   DESTINATION
   ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
