cmake_minimum_required(VERSION 3.5.0)
project(rct_image_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rct_optimizations REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/image_observation_finder.cpp
  src/${PROJECT_NAME}/aruco_finder.cpp
  src/${PROJECT_NAME}/circle_detector.cpp
  src/${PROJECT_NAME}/modified_circle_grid_target.cpp
)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rct_optimizations_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${rct_optimizations_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test
  src/observation_finder_tests.cpp
)

target_link_libraries(${PROJECT_NAME}_test
#  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Package ##
#############

ament_export_include_directories(
    include)

ament_export_libraries(
    ${PROJECT_NAME})

ament_export_dependencies(
    rct_optimizations
    OpenCV)

ament_package()

#############
## Install ##
#############

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
