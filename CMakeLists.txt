cmake_minimum_required(VERSION 3.0.2)
project(hrii_robothon_byod)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  hrii_gri_interface
  hrii_robothon_msgs
  hrii_trajectory_planner
  message_generation
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
   hrii_trajectory_planner
   hrii_gri_interface
   hrii_robothon_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main_byod src/main_byod.cpp)
set_target_properties(${PROJECT_NAME}_main_byod PROPERTIES OUTPUT_NAME "main_byod")
add_dependencies(${PROJECT_NAME}_main_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_main_byod ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_homing_byod src/tasks/homing_byod.cpp)
set_target_properties(${PROJECT_NAME}_homing_byod PROPERTIES OUTPUT_NAME "homing_byod")
add_dependencies(${PROJECT_NAME}_homing_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_homing_byod ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_battery_opening_byod src/tasks/battery_opening_byod.cpp)
set_target_properties(${PROJECT_NAME}_battery_opening_byod PROPERTIES OUTPUT_NAME "battery_opening_byod")
add_dependencies(${PROJECT_NAME}_battery_opening_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_battery_opening_byod ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_power_on_voltmeter_byod src/tasks/power_on_voltmeter_byod.cpp)
set_target_properties(${PROJECT_NAME}_power_on_voltmeter_byod PROPERTIES OUTPUT_NAME "power_on_voltmeter_byod")
add_dependencies(${PROJECT_NAME}_power_on_voltmeter_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_power_on_voltmeter_byod ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_probe_circuit_byod src/tasks/probe_circuit_byod.cpp)
set_target_properties(${PROJECT_NAME}_probe_circuit_byod PROPERTIES OUTPUT_NAME "probe_circuit_byod")
add_dependencies(${PROJECT_NAME}_probe_circuit_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_probe_circuit_byod ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_battery_screening_byod src/tasks/battery_screening_byod.cpp)
set_target_properties(${PROJECT_NAME}_battery_screening_byod PROPERTIES OUTPUT_NAME "battery_screening_byod")
add_dependencies(${PROJECT_NAME}_battery_screening_byod ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_battery_screening_byod ${catkin_LIBRARIES})