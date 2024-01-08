cmake_minimum_required(VERSION 3.0.2)
project(pivot_base)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# Declare a catkin package
catkin_package(

    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES pivot_base
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
        rospy
)

# Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Create Cpp executable
add_executable(pivot_base_node src/pivot_base.cpp)
target_link_libraries(pivot_base_node ${catkin_LIBRARIES})

add_executable(move_navigation_node src/move_navigation.cpp)
target_link_libraries(move_navigation_node ${catkin_LIBRARIES})

add_executable(unit_test_node src/unit_test.cpp)
target_link_libraries(unit_test_node ${catkin_LIBRARIES})

add_executable(pivot_planner_node src/pivot_planner.cpp)
target_link_libraries(pivot_planner_node ${catkin_LIBRARIES})

# Install Cpp executable
install(TARGETS pivot_base_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS move_navigation_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS unit_test_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS pivot_planner_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install Python executable
#install(PROGRAMS scripts/vgraph_planner.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})