cmake_minimum_required(VERSION 2.4.6)
project(duo3d_driver)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
             image_transport
             roscpp
             rosconsole
             sensor_msgs
             dynamic_reconfigure
             tf2_ros
             pcl_conversions
             pcl_ros
             cv_bridge
)

generate_dynamic_reconfigure_options(cfg/Duo3D.cfg)

catkin_package()

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS} 
                    ${PCL_INCLUDE_DIRS}
)
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

add_definitions(-std=c++11)

add_executable(duo3d_driver src/duo3d_driver.cpp)
add_dependencies(duo3d_driver ${PROJECT_NAME}_gencfg)
target_link_libraries(duo3d_driver 
                      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDUO.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDense3DMT.so
                      ${catkin_LIBRARIES}
)
