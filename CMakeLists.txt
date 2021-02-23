cmake_minimum_required(VERSION 3.0.2)
project(simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    xmlrpcpp
    tf2
    tf2_ros
)


##########################
## Catkin configuration ##
##########################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs xmlrpcpp
)

###########
## Build ##
###########

## Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (
    WEBOTS_LIB
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    lib/sensors/include
    lib/steering/include
    $ENV{WEBOTS_HOME}/include/controller/c
    $ENV{WEBOTS_HOME}/include/controller/cpp
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(sensors lib/sensors/src/lidar.cpp lib/sensors/src/gps.cpp lib/sensors/src/inertialUnit.cpp lib/sensors/src/accelerometer.cpp lib/sensors/src/gyro.cpp lib/sensors/src/wheelOdometry.cpp)
add_library(steering lib/steering/src/diffSteering.cpp)
target_link_libraries(sensors ${WEBOTS_LIB} ${catkin_LIBRARIES})
target_link_libraries(steering ${WEBOTS_LIB} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(full_controller src/full_controller.cpp)
target_link_libraries(full_controller sensors steering ${WEBOTS_LIB} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS launch/webots_launcher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS full_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS sensors
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS steering
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY lib/sensors/include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY lib/steering/include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



## Mark all launch files for installation
install(DIRECTORY launch/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark all world files for installation
install(DIRECTORY worlds/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

