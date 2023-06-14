# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(realsense_gazebo)

find_package(catkin REQUIRED COMPONENTS sensor_msgs)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_plugins REQUIRED)

if(DEFINED ENV{GZ_VERSION})
    find_package(ros_ign REQUIRED)
endif()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
#catkin_lint: ignore_once link_directory
link_directories(${catkin_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS} ${gazebo_plugins_LIBRARY_DIRS})

add_library(gzrs_depth_camera src/gazebo_ros_depth_camera.cpp)
target_link_libraries(gzrs_depth_camera ${catkin_LIBRARIES} ${gazebo_dev_LIBRARIES} ${gazebo_plugins_LIBRARIES} DepthCameraPlugin)
target_include_directories(gzrs_depth_camera PRIVATE ${gazebo_dev_INCLUDE_DIRS} ${gazebo_plugins_INCLUDE_DIRS})

install(TARGETS gzrs_depth_camera
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
    nodes/extrinsics_pub
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

if (CATKIN_ENABLE_TESTING)
    find_package(roslint REQUIRED)

    # catkin_lint - checks validity of package.xml and CMakeLists.txt
    # ROS buildfarm calls this without any environment and with empty rosdep cache,
    # so we have problems reading the list of packages from env
    # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
    if(DEFINED ENV{ROS_HOME})
        #catkin_lint: ignore_once env_var
        set(ROS_HOME "$ENV{ROS_HOME}")
    else()
        #catkin_lint: ignore_once env_var
        set(ROS_HOME "$ENV{HOME}/.ros")
    endif()

    #catkin_lint: ignore_once env_var
    if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
        roslint_custom(catkin_lint -W2 --ignore missing_depend .)
    endif()

    set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
    roslint_python(
        nodes/extrinsics_pub
    )

    set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
    roslint_cpp(
        src/gazebo_ros_depth_camera.cpp
    )

    roslint_add_test()
endif()
