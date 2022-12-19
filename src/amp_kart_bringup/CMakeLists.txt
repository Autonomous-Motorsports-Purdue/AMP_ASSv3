cmake_minimum_required(VERSION 3.5)
project(amp_kart_bringup)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch params
    DESTINATION share/${PROJECT_NAME})

ament_package()
