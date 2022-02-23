cmake_minimum_required(VERSION 3.10)
project(elevation_mapping_demos)

# Use C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

# include_directories(
#   include
#   # ${PCL_INCLUDE_DIRS}
#   # ${EIGEN3_INCLUDE_DIRS}
#   # ${KINDR_ROS_INCLUDE_DIRS}
# )

# link_directories(
#   ${PCL_LIBRARY_DIRS}
#   ${EIGEN3_LIBRARY_DIRS}
# )

# add_definitions(${PCL_DEFINITIONS})

install(
  DIRECTORY
    launch
    config
    rviz2
  DESTINATION 
    share/${PROJECT_NAME}
)

install(PROGRAMS  # TODO(SivertHavso): replace with ament_python_install_package
  scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME})

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
#   DIRECTORY
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


# cmake_minimum_required(VERSION 3.5.1)
# project(elevation_mapping_demos)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   elevation_mapping
#   grid_map_rviz_plugin
#   rviz
# )


# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if you package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need

# catkin_package(
#   #INCLUDE_DIRS include
#   #LIBRARIES
#   CATKIN_DEPENDS
#     elevation_mapping
#     grid_map_rviz_plugin
#     rviz
#   #DEPENDS
# )

# #############
# ## Install ##
# #############

# install(DIRECTORY config doc launch rviz sample_data
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(PROGRAMS scripts/tf_to_pose_publisher.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ##############
# ##   Test   ##
# ##############
# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED
#     COMPONENTS
#     roslaunch
#     )
#   catkin_add_gtest(test_${PROJECT_NAME}
#     test/empty_test.cpp
#   )

#   target_include_directories(test_${PROJECT_NAME}
#     PRIVATE
#       include
#     SYSTEM PUBLIC
#       ${catkin_INCLUDE_DIRS}
#   )

#   target_link_libraries(test_${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     gtest_main
#   )
#   roslaunch_add_file_check(launch)

#   ###################
#   ## Code_coverage ##
#   ###################
#   find_package(cmake_code_coverage QUIET)
#   if(cmake_code_coverage_FOUND)
#     add_gtest_coverage(
#       TEST_BUILD_TARGETS
#         test_${PROJECT_NAME}
#     )
#   endif()
# endif()