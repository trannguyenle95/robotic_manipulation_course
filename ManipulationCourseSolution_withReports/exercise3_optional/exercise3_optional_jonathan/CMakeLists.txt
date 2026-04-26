cmake_minimum_required(VERSION 2.8.3)
project(exercise3_optional)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS xacro
        moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface)
find_package(PCL REQUIRED)
find_package(OpenCV)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
catkin_package()

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(pose_estimation ${catkin_EXPORTED_TARGETS})

install(TARGETS pose_estimation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

file(GLOB xacro_files model/robots/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET media_files INORDER)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY model DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
