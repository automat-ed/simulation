cmake_minimum_required(VERSION 3.0.2)
project(simulation)


find_package(catkin REQUIRED
    COMPONENTS
    rospy
)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS launch/webots_launcher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

