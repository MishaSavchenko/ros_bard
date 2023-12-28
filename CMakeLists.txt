cmake_minimum_required(VERSION 3.8)
project(ros_bard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(graphiz REQUIRED)
INCLUDE_DIRECTORIES(SYSTEM "/usr/include/graphviz/")
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(cgraph IMPORTED_TARGET ${PROJECT_NAME})
# pkg_check_modules(gvc IMPORTED_TARGET ${PROJECT_NAME})
# find_package(cgraph REQUIRED)

# find_package(libgvc REQUIRED)
# find_package(graphviz REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rmw_test scripts/rmw_test.cpp)
ament_target_dependencies(rmw_test rclcpp)

add_executable(graphviz_test scripts/graphviz_test.cpp)
# ament_target_dependencies(graphviz_test cgraph gvc)
target_link_libraries(graphviz_test cgraph gvc)


install(TARGETS
  graphviz_test
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
