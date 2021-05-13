cmake_minimum_required(VERSION 3.0.2)
project(infantry_detection)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  std_msgs
  sensor_msgs
  infantry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

FILE(GLOB STREAM src/** )

SET(TARGET_NAME ${PROJECT_NAME}_node)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/Armor armor_SRCS)
add_library(armorLib ${armor_SRCS})
target_link_libraries(armorLib ${OpenCV_LIBS})

aux_source_directory(src/AngleSolver angle_solver_SRCS)
add_library(AngleSolverLib ${angle_solver_SRCS})

add_executable(${TARGET_NAME} src/Main/main.cpp)
add_dependencies(${TARGET_NAME} infantry_msgs_gencpp)
target_link_libraries(
    ${TARGET_NAME}  
    ${catkin_LIBRARIES}
    armorLib
    AngleSolverLib
)

add_executable(serial_test_node src/serial_test.cpp)
add_dependencies(serial_test_node infantry_msgs_gencpp)
target_link_libraries(
  serial_test_node 
    ${catkin_LIBRARIES}
)