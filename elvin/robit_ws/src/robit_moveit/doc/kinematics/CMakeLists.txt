#add_executable(kinematic_model src/kinematic_model.cpp)
#target_link_libraries(kinematic_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS roscpp moveit_ros_planning_interface)
add_executable(robit_move_group_interface src/robit_move_group_interface.cpp)
target_link_libraries(robit_move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
## target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## install(TARGETS kinematic_model_tutorial ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(TARGETS kinematic_model robit_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS robit_move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
