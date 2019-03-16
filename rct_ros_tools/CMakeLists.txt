cmake_minimum_required(VERSION 3.5)
project(rct_ros_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rct_image_tools REQUIRED)
find_package(rct_optimizations REQUIRED)

# TODO: Export these to a different package dedicated to 'ros tools'
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)


###########
## Build ##
###########

# Create a library for loading and saving "data sets": directory based
# archives of poses and images that can be reloaded for testing and
# development purposes.
add_library(${PROJECT_NAME}_data_loader
  src/data_loader/data_set.cpp
  src/data_loader/parameter_loaders.cpp)

target_include_directories(${PROJECT_NAME}_data_loader PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rct_image_tools_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_data_loader
 yaml-cpp
 ${OpenCV_LIBRARIES}
 ${rclcpp_LIBRARIES}
 ${std_srvs_LIBRARIES}

)

# Executable for collecting data sets via subscribers and triggered with services
# See readme (TODO: Write a readme)
add_executable(${PROJECT_NAME}_cmd src/command_line_cal.cpp)

set_target_properties(${PROJECT_NAME}_cmd PROPERTIES OUTPUT_NAME command_line_data_collection PREFIX "")

target_include_directories(${PROJECT_NAME}_cmd PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${PROJECT_NAME}_cmd
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}_data_loader
)

ament_export_include_directories(
    include
    ${rct_image_tools_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS})

ament_export_libraries(
    ${PROJECT_NAME}_data_loader
    yaml-cpp)

ament_export_dependencies(
    rclcpp)

ament_package()

#############
## Testing ##
#############

#############
## Install ##
#############

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS ${PROJECT_NAME}_data_loader EXPORT ${PROJECT_NAME}_data_loader
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_cmd EXPORT ${PROJECT_NAME}_cmd
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
