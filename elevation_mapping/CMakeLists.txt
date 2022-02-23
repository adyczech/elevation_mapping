cmake_minimum_required(VERSION 3.10)
project(elevation_mapping)

# Use C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find_package(kindr_ros REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${KINDR_ROS_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Our custom pcl type with precompiled template instantiations.
ament_auto_add_library(${PROJECT_NAME}_pcl_types
  src/PointXYZRGBConfidenceRatio.cpp
)

ament_auto_add_library(${PROJECT_NAME}
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/input_sources/Input.cpp
  src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

target_link_libraries(${PROJECT_NAME}
 ${PCL_LIBRARIES})
#  ${PROJECT_NAME}_pcl_types

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_prio_node src/${PROJECT_NAME}_node_prio_threads.cpp)
target_link_libraries(${PROJECT_NAME}_prio_node
  ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # ignore copyright checks
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_copyright
  )

  # run cpplint without copyright filter
  find_package(ament_cmake_cpplint)
  ament_cpplint(
    FILTERS -legal/copyright
  )

  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(test)
endif()

# install(
#   TARGETS
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION
#     lib
#   LIBRARY DESTINATION
#     lib
#   RUNTIME DESTINATION
#     lib/${PROJECT_NAME}
# )

ament_auto_package()
