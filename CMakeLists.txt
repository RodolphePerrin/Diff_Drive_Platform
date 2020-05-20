cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(diff_drive)

 Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################

catkin_package()

# catkin_package(
#   INCLUDE_DIRS
#     "include"
#   LIBRARIES
#     ${PROJECT_NAME}
#   CATKIN_DEPENDS
#     nodelet
#     tf
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Package locations should be listed before other locations
# include_directories(
#   "include/"
#   ${EXTERNAL_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

## Build the botn library

# add_library(${PROJECT_NAME}
#   "src/driver.cpp"
# )
#
# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
#
# target_link_libraries(${PROJECT_NAME}
#   ${EXTERNAL_LIBS}
#   ${catkin_LIBRARIES}
# )
