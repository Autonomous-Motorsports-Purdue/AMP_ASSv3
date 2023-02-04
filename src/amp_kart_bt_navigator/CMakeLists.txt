cmake_minimum_required(VERSION 3.5)
project(amp_kart_bt_navigator)
find_package(ament_cmake REQUIRED)

install(DIRECTORY behavior_trees
    DESTINATION share/${PROJECT_NAME})

ament_package()
