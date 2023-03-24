cmake_minimum_required(VERSION 3.5)
project(amp_kart_description)
find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes launch rviz urdf
    DESTINATION share/${PROJECT_NAME})

ament_package()
