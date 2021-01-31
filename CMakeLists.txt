cmake_minimum_required(VERSION 3.0.2)
project(simulation)


find_package(catkin REQUIRED
    COMPONENTS
    rospy
)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS controllers/robot_controller/robot_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
