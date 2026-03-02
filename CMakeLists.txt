cmake_minimum_required(VERSION 3.0.2)
project(lys_visp_demo)

## ====== ASan BEGIN ======
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(
  -fsanitize=address
  -fno-omit-frame-pointer
  -g
)

add_link_options(
  -fsanitize=address
)
## ====== ASan END ======

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  visp_ros
  realsense2_camera
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  visp_bridge
)

# 根据您的查找结果，头文件在/usr/include/fl/
# 库文件在/usr/lib/x86_64-linux-gnu/libfuzzylite.so
set(fuzzylite_INCLUDE_DIRS "/usr/include")
set(fuzzylite_LIBRARIES "/usr/lib/x86_64-linux-gnu/libfuzzylite.so")
message(STATUS "fuzzylite include dirs: ${fuzzylite_INCLUDE_DIRS}")
message(STATUS "fuzzylite libraries: ${fuzzylite_LIBRARIES}")

find_package(VISP REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(ruckig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
#find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#添加控制选项
option(BUILD_VISPCTRL "Build visp_ctrl.cpp" ON)

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
   AprilTagCorners.msg #添加自定义消息
   HomogeneousTransform.msg
   PixelCoordinates.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SendCommand.srv
#   Service2.srv
)

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
   geometry_msgs #确保生成依赖
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
 CATKIN_DEPENDS  message_runtime  roscpp rospy std_msgs geometry_msgs
#  INCLUDE_DIRS include
#  LIBRARIES lys_visp_demo
#  CATKIN_DEPENDS roscpp visp_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${REALSENSE2_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${fuzzylite_INCLUDE_DIRS} 
)

add_library(mpc_ibvs_controllerV1
  src/mpc_ibvs_controllerV1.cpp
)

add_library(mpc_ibvs_controllerV2
  src/mpc_ibvs_controllerV2.cpp
)

add_library(mpc_ibvs_controllerV3
  src/mpc_ibvs_controllerV3.cpp
)

add_library(mpc_ibvs_controllerV4
  src/mpc_ibvs_controllerV3.cpp
)

add_executable(camera_sub src/camera_sub.cpp)
add_executable(visp_ctrl  src/visp_ctrl.cpp )
add_executable(jr603_IBVS src/jr603_IBVS.cpp)
add_executable(test_visp_realsence src/test_visp_realsence.cpp)
add_executable(jr603_ibvs_ctrl src/jr603_ibvs_ctrl.cpp)
add_executable(jr603_velocity_with_otg src/jr603_velocity_with_otg.cpp)
add_executable(jr603_pos_with_otg src/jr603_pos_with_otg.cpp)

#add_executable(mpc_ibvs_controllerV1 src/mpc_ibvs_controllerV1.cpp)

add_executable(jr603_mpc_ibvs_ctrlV0 src/jr603_mpc_ibvs_ctrlV0.cpp)

add_executable(jr603_mpc_ibvs_ctrlV4 src/jr603_mpc_ibvs_ctrlV4.cpp) #不加速度和视野约束
add_executable(jr603_mpc_ibvs_ctrlV5 src/jr603_mpc_ibvs_ctrlV5.cpp) #添加速度和视野约束，但是mpc求解的仍然是无约束优化问题
add_executable(jr603_mpc_ibvs_ctrlV6 src/jr603_mpc_ibvs_ctrlV6.cpp) #添加了速度和视野约束，使用OSQP求解器

add_executable(jr603_fd_ibvs_ctrlV1 src/jr603_fd_ibvs_ctrlV1.cpp)   #模糊方向速度控制

set_target_properties(visp_ctrl PROPERTIES EXCLUDE_FROM_ALL FALSE)

target_link_libraries(jr603_ibvs_ctrl
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            )

target_link_libraries(jr603_fd_ibvs_ctrlV1
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            ${fuzzylite_LIBRARIES}
            )
                     
target_link_libraries(jr603_mpc_ibvs_ctrlV0
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            )
            
target_link_libraries(mpc_ibvs_controllerV1
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(mpc_ibvs_controllerV2
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(mpc_ibvs_controllerV3
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(mpc_ibvs_controllerV4
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)
            
target_link_libraries(jr603_mpc_ibvs_ctrlV4
  			mpc_ibvs_controllerV2      # 链接自己的库
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(jr603_mpc_ibvs_ctrlV5
  			mpc_ibvs_controllerV3      # 链接自己的库
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(jr603_mpc_ibvs_ctrlV6
  			mpc_ibvs_controllerV4      # 链接自己的库
  			${catkin_LIBRARIES}
  			${VISP_LIBRARIES}
  			OsqpEigen::OsqpEigen
  			${EIGEN3_LIBRARIES}        # 添加Eigen3
  			#${OpenCV_LIBS}             # 添加OpenCV
)

target_link_libraries(test_visp_realsence
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            ${realsense2_camera_LIBRARIES}
            )

target_link_libraries(jr603_IBVS
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            ${REALSENSE2_LIBRARIES}
            )

target_link_libraries(camera_sub 
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            )

target_link_libraries(visp_ctrl
            ${catkin_LIBRARIES}
            ${VISP_LIBRARIES}
            )

target_link_libraries(jr603_velocity_with_otg
	    ${catkin_LIBRARIES}
	    ruckig::ruckig
	    )
target_link_libraries(jr603_pos_with_otg
	    ${catkin_LIBRARIES}
	    ruckig::ruckig
	    )
	    
# 如果从源码构建fuzzylite，则添加依赖
if(TARGET fuzzylite_built)
  add_dependencies(jr603_fuzzy_ibvs_ctrl fuzzylite_built)
  message(STATUS "Added fuzzylite build dependency to jr603_fuzzy_ibvs_ctrl")
endif()


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lys_visp_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/lys_visp_demo_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/webcam_pub.py
  scripts/apriltag_detect.py
  scripts/coords_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lys_visp_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
