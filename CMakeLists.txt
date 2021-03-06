cmake_minimum_required(VERSION 3.1.0)
project(follow_me_flie_ros)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
#Include QT5
set(CMAKE_PREFIX_PATH "/usr/include/x86_64-linux-gnu/qt5")
#Always build debug
set(CMAKE_BUILD_TYPE Debug)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)
# Find the QtWidgets library
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Multimedia CONFIG REQUIRED)
find_package(Qt5MultimediaWidgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
set(qt5_components
   Qt5::Widgets
   Qt5::Multimedia
   Qt5::MultimediaWidgets
   Qt5::Gui
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RawPacket.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES follow_me_flie_ros
  CATKIN_DEPENDS roscpp message_runtime std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

## Declare crazy_flie_library
add_library(${PROJECT_NAME}_lib
  src/text_logger.cpp
  src/math/functions.cpp
  src/math/kalman_filter.cpp
  src/math/lowpass.cpp
  src/math/pi_controller.cpp
  src/math/pid_controller.cpp
  src/math/types.cpp
  src/crazyflie/crazy_flie_commander.cpp
  src/crazyflie/crazy_flie.cpp
  src/crazyflie/crtp_packet.cpp
  src/crazyflie/radio_dongle.cpp
  src/crazyflie/toc_log.cpp
  src/crazyflie/toc_parameter.cpp
  src/crazyflie/raw_packet.cpp
  src/qt_util.cpp
  src/opencv_utils.cpp
  src/imageprocess/ball_kalman_filter_2d.cpp
  src/imageprocess/ball_kalman_filter_3d.cpp
  src/imageprocess/camera.cpp
  src/imageprocess/coordinate_conversions.cpp
  src/imageprocess/extract_color.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
    ${qt5_components}
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

set(GUI_APP_SRCS
    src/main.cpp
    src/gui/actual_values_model.cpp
    src/gui/camera_view_painter.cpp
    src/gui/main_window.cpp
    src/gui/main_window.ui
    src/gui/parameter_model.cpp
    src/gui/set_point_dialog.cpp
    src/gui/set_point_dialog.ui
    src/gui/table_model_base.cpp
    src/gui/tracking_color.cpp
        src/crazyflie/PacketHandler.cpp src/crazyflie/PacketHandler.h)
add_executable(${PROJECT_NAME}_node ${GUI_APP_SRCS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
find_package(OpenCV 3 CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   Qt5::Widgets
   ${PROJECT_NAME}_lib
   ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test 
        test/test_main.cpp
        test/camera_test.cpp
        test/crtppacket_test.cpp
        test/delay_test.cpp
        test/double_buffer_test.cpp
        test/logic_test.cpp
        test/math_function_test.cpp
        test/pid_controller_test.cpp
        test/stl_utils_test.cpp
        test/toc_log_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}_lib
        ${OpenCV_LIBRARIES}
    )
endif()
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
