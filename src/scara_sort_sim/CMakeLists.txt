cmake_minimum_required(VERSION 3.8)
project(scara_sort_sim)

find_package(ament_cmake REQUIRED)

# No code to compile: just copy these folders into the install space
install(DIRECTORY urdf worlds models config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
