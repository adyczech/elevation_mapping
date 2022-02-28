# find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
    test_postprocessor_pool
    # postprocessing/postprocessor.test
    postprocessing/test_postprocessor.launch.py
    postprocessing/main.cpp
    postprocessing/PostprocessorTest.cpp
)

add_launch_test(
  postprocessing/test_postprocessor.launch.py
)

if(TARGET test_postprocessor_pool)
  ament_target_dependencies(test_postprocessor_pool "rclcpp")
  target_link_libraries(test_postprocessor_pool ${PROJECT_NAME})

  install(  # TODO(SivertHavso): actually necessary?
    TARGETS
      test_postprocessor_pool
    RUNTIME DESTINATION
      test
  )
  install(
    FILES
      postprocessing/postprocessor_pipeline.yaml
    DESTINATION
      test
  )
endif()
