cmake_minimum_required(VERSION 3.5)
project(adaptive_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_find_build_dependencies()

set(msg_files
  "msg/ClusterArray.msg"
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

if(NOT COMMAND rosidl_get_typesupport_target)
  function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)
    if(NOT TARGET ${generate_interfaces_target})
      message(FATAL_ERROR
        "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
    endif()

    set(output_target "${generate_interfaces_target}__${typesupport_name}")

    if(NOT TARGET ${output_target})
      # CMake if() evaluates strings ending in `-NOTFOUND` as false
      set(output_target "${output_target}-NOTFOUND")
    endif()

    set("${var}" "${output_target}" PARENT_SCOPE)
  endfunction()
endif()

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/adaptive_clustering.cpp
)

target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# export information to downstream packages
ament_auto_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
  INSTALL_TO_SHARE param
)

# tests
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  #ament_add_gtest(${PROJECT_NAME}_gtest test/my_test.cpp)
  #target_link_libraries(${PROJECT_NAME}_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces})

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
