cmake_minimum_required(VERSION 3.10)
project(buff_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        control_toolbox
        controller_interface
        geometry_msgs
        hardware_interface
        pluginlib
        realtime_tools
        roscpp
        sensor_msgs
        std_msgs
        tf
)

###################################
## catkin specific configuration ##
###################################


catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_interface
        hardware_interface
        pluginlib
        control_toolbox
        std_msgs
        realtime_tools
        geometry_msgs
        tf
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(buff_controller src/buff_controller.cpp)

#add_dependencies(${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#)

target_link_libraries(buff_controller
        ${catkin_LIBRARIES}
        ${controller_interface_LIBRARIES}
        ${hardware_interface_LIBRARIES}
)
#############
## Install ##
#############

# Mark resource files for installation
install(
        DIRECTORY urdf meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS buff_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
        FILES buff_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
