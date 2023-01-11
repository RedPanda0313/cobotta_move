cmake_minimum_required(VERSION 3.0.2)
project(cobotta_move)

add_definitions(-std=c++14)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set ( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/modules/  )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED Eigen3 kdl_parser urdf)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)
include_directories(include ${catkin_INCLUDE_DIRS}  )
include_directories(include ${EIGEN3_INCLUDE_DIRS}  )


add_executable(move_test src/main.cpp 
src/kdl/kdl_parse.cpp
src/kdl/kdl_ik.cpp
src/kdl/kdl_base.cpp
src/kdl/chainiksolverpos_my.cpp
src/cobotta_move/callbacks.cpp 
src/cobotta_move/robot_states.cpp
src/cobotta_move/robot_rviz.cpp
src/cobotta_move/robot_data.cpp
src/cobotta_move/robot_move.cpp
src/cobotta_move/robot_init.cpp  
src/cobotta_move/robot_move_lf.cpp 
src/cobotta_move/robot_move_ik.cpp 
src/cobotta_move/robot_move_ik_pose.cpp 
src/cobotta_move/robot_move_ik_roll.cpp 
src/cobotta_move/robot_move_joint.cpp 
src/cobotta_move/cobotta_move.cpp 
src/cobotta_move/cobotta_param.cpp 
src/utility/ros_calc.cpp 
)
target_link_libraries(move_test ${catkin_LIBRARIES}  )

add_executable(move_test2 src/main2.cpp 
src/kdl/kdl_parse.cpp
src/kdl/kdl_ik.cpp
src/kdl/kdl_base.cpp
src/kdl/chainiksolverpos_my.cpp
src/cobotta_move/callbacks.cpp 
src/cobotta_move/robot_states.cpp
src/cobotta_move/robot_rviz.cpp
src/cobotta_move/robot_data.cpp
src/cobotta_move/robot_move.cpp
src/cobotta_move/robot_init.cpp  
src/cobotta_move/robot_move_lf.cpp 
src/cobotta_move/robot_move_ik.cpp 
src/cobotta_move/robot_move_ik_pose.cpp 
src/cobotta_move/robot_move_ik_roll.cpp 
src/cobotta_move/robot_move_joint.cpp 
src/cobotta_move/cobotta_move.cpp 
src/cobotta_move/cobotta_param.cpp 
src/utility/ros_calc.cpp 
)
target_link_libraries(move_test2 ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} 
src/kdl/kdl_parse.cpp
src/kdl/kdl_ik.cpp
src/kdl/kdl_base.cpp
src/kdl/chainiksolverpos_my.cpp
src/cobotta_move/callbacks.cpp 
src/cobotta_move/robot_states.cpp
src/cobotta_move/robot_rviz.cpp
src/cobotta_move/robot_data.cpp
src/cobotta_move/robot_move.cpp
src/cobotta_move/robot_init.cpp  
src/cobotta_move/robot_move_lf.cpp 
src/cobotta_move/robot_move_ik.cpp 
src/cobotta_move/robot_move_ik_pose.cpp 
src/cobotta_move/robot_move_ik_roll.cpp 
src/cobotta_move/robot_move_joint.cpp 
src/cobotta_move/cobotta_move.cpp 
src/cobotta_move/cobotta_param.cpp 
src/utility/ros_calc.cpp 
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# headers
install(DIRECTORY include/cobotta_move/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
# headers
install(DIRECTORY include/kdl/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

# other files
foreach(dir launch config)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()