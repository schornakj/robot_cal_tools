cmake_minimum_required(VERSION 3.5.0)
project(rct_optimizations)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  # Utilities
  src/${PROJECT_NAME}/eigen_conversions.cpp
  # Optimizations (monocular camera)
  src/${PROJECT_NAME}/extrinsic_camera_on_wrist.cpp
  src/${PROJECT_NAME}/extrinsic_static_camera.cpp
  # Optimizations (multiple cameras)
  src/${PROJECT_NAME}/extrinsic_multi_static_camera.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_only.cpp
  src/${PROJECT_NAME}/extrinsic_multi_static_camera_wrist_only.cpp
  # Optimizations (3D sensor)
  src/${PROJECT_NAME}/extrinsic_3d_camera_on_wrist.cpp
  # Optimizations (Experimental) - Intrinsic
  src/${PROJECT_NAME}/camera_intrinsic.cpp
  src/${PROJECT_NAME}/pnp.cpp
  src/${PROJECT_NAME}/multi_camera_pnp.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
)

#############
## Testing ##
#############
if(AMENT_ENABLE_TESTING)
  # Build the test support library
  add_library(${PROJECT_NAME}_test_support
    test/src/observation_creator.cpp
    test/src/utilities.cpp)
  target_link_libraries(${PROJECT_NAME}_test_support ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test_support PUBLIC test/include)

  # The actual tests...
  ament_add_gtest(${PROJECT_NAME}_conversion_tests test/conversion_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_conversion_tests ${PROJECT_NAME}_test_support)

  ament_add_gtest(${PROJECT_NAME}_extrinsic_multi_static_camera_tests test/extrinsic_multi_static_camera_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_extrinsic_multi_static_camera_tests ${PROJECT_NAME}_test_support)

  ament_add_gtest(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests test/extrinsic_camera_on_wrist_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests ${PROJECT_NAME}_test_support)
endif()


#############
## Package ##
#############

ament_export_include_directories(
    include
    ${CERES_INCLUDE_DIRS})

ament_export_libraries(
    ${PROJECT_NAME}
    ${CERES_LIBRARIES})

ament_export_dependencies(
    Ceres)

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
