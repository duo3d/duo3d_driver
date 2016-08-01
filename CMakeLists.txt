cmake_minimum_required(VERSION 2.4.6)
#cmake_policy(SET CMP0003 NEW)
project(duo3d_driver)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

message("Build: " ${CMAKE_BUILD_TYPE})

#find_package(PCL REQUIRED)

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

#find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)

#message("OpenCV: " ${OpenCV_DIR})
#message("PCL: " ${PCL_DIR})

generate_dynamic_reconfigure_options(
  cfg/Duo3D.cfg
)

catkin_package()

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS} 
                    ${PCL_INCLUDE_DIRS}
)

#message(WARNING "Path: " ${CMAKE_CURRENT_SOURCE_DIR})

#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/)

#link_directories(${OpenCV_LIBRARY_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

add_definitions(-std=c++11)

add_executable(duo3d_driver src/duo3d_driver.cpp)

target_link_libraries(duo3d_driver 
                      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDUO.so
                      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDense3DMT.so
                      ${catkin_LIBRARIES}
)

