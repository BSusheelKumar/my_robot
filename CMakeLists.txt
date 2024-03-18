cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  my_robot
  SHARED
  include/my_robot_system.cpp
)
target_link_libraries(my_robot PUBLIC serial)

target_compile_features(my_robot PUBLIC cxx_std_17)
target_include_directories(my_robot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:my_robot>
)
ament_target_dependencies(
  my_robot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "MY_ROBOT_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface my_robot.xml)


install(TARGETS my_robot
  EXPORT export_my_robot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

install(
  DIRECTORY config description launch world
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})



install(PROGRAMS scripts/py_node.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/camera_detect.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
