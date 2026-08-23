cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY behavior_trees config launch maps src rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    src/nav2_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    navigation_pipeline_test
    test/test_navigation_pipeline.py
  )
endif()

ament_package()
