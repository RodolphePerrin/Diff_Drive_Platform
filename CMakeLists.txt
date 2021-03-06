cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(diff_drive)

##Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  nav_msgs
  geometry_msgs
  gazebo_dev
  message_generation
  gazebo_msgs
  tf

)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    "Encoders.msg"
    )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################



catkin_package(
   INCLUDE_DIRS
     "include"
   LIBRARIES
     ${PROJECT_NAME}
     diff_drive_gazebo
  CATKIN_DEPENDS
     gazebo_plugins
     nodelet
     tf
 )

###########
## Build ##
###########


## Specify additional locations of header files
## Package locations should be listed before other locations
 include_directories(
    "include/"
    ${EXTERNAL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
 )


# Build the odometry node for differential drive robot.

add_executable(odometry
  "src/odometry.cpp"
  "src/odometry/state.cpp"
  "src/odometry/encoders.cpp"
  "src/odometry/imu.cpp"
)

add_dependencies(odometry
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(odometry
  ${catkin_LIBRARIES}
)



## Build library diff_drive_gazebo

add_library(diff_drive_gazebo
  "src/diff_drive.cpp"
)

target_link_libraries(diff_drive_gazebo 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
