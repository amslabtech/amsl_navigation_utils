cmake_minimum_required(VERSION 3.0.2)
project(amsl_navigation_utils)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    tf
    pcl_conversions
    ndt_omp
    amsl_navigation_msgs
)
find_package(PCL 1.8.0 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES amsl_navigation_utils
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINISIONS})

add_executable(footprint_publisher src/footprint_publisher.cpp)
target_link_libraries(footprint_publisher ${catkin_LIBRARIES})

add_executable(initialpose_publisher src/initialpose_publisher.cpp)
target_link_libraries(initialpose_publisher ${catkin_LIBRARIES})

add_executable(initialpose_adjuster src/initialpose_adjuster.cpp)
target_link_libraries(initialpose_adjuster ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_cloud_downsampler src/map_cloud_downsampler.cpp)
add_dependencies(map_cloud_downsampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_cloud_downsampler ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odom_broadcaster src/odom_broadcaster.cpp)
target_link_libraries(odom_broadcaster ${catkin_LIBRARIES})

add_executable(odom_to_pose src/odom_to_pose.cpp)
add_dependencies(odom_to_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_to_pose ${catkin_LIBRARIES})

add_executable(scan_invertor src/scan_invertor.cpp)
target_link_libraries(scan_invertor ${catkin_LIBRARIES})

# add_executable(pid_controller src/pid_controller.cpp)
# target_link_libraries(pid_controller ${catkin_LIBRARIES})

#############
## Testing ##
#############
# if(CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     find_package(roslint REQUIRED)
#     roslint_cpp()
#     roslint_add_test()
# endif()