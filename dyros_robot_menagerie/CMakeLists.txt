cmake_minimum_required(VERSION 3.14)
project(dyros_robot_menagerie LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release )
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(mujoco_ros_sim         REQUIRED)
find_package(dyros_robot_controller REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(Eigen3                 REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(geometry_msgs          REQUIRED)
find_package(sensor_msgs            REQUIRED)

include_directories(include)

# ===============================================
# ===================== FR3 =====================

add_library(fr3_controller_plugins SHARED
  src/fr3/controller.cpp
  src/fr3/robot_data.cpp
)
ament_target_dependencies(fr3_controller_plugins
  rclcpp 
  pluginlib 
  mujoco_ros_sim
)

target_link_libraries(fr3_controller_plugins
  dyros_robot_controller::RobotData
  dyros_robot_controller::RobotController
)

install(TARGETS fr3_controller_plugins
        LIBRARY DESTINATION lib
)

install(PROGRAMS    ${PROJECT_NAME}/fr3/ros2_qt_gui.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME      FR3ControllerQT
)

# # ===================== Husky =====================
# mrs_add_controller(husky_controller
#   src/husky/controller.cpp
#   src/husky/robot_data.cpp
# )

# target_link_libraries(husky_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(husky_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/husky/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      HuskyControllerQT
# )
# # ===================== XLS =====================
# mrs_add_controller(xls_controller
#   src/xls/controller.cpp
#   src/xls/robot_data.cpp
# )

# target_link_libraries(xls_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(xls_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/xls/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      XLSControllerQT
# )
# # ===================== PCV =====================
# mrs_add_controller(pcv_controller
#   src/pcv/controller.cpp
#   src/pcv/robot_data.cpp
# )

# target_link_libraries(pcv_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(pcv_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/pcv/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      PCVControllerQT
# )
# # ===================== FR3 Husky =====================
# mrs_add_controller(fr3_husky_controller
#   src/fr3_husky/controller.cpp
#   src/fr3_husky/robot_data.cpp
# )

# target_link_libraries(fr3_husky_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(fr3_husky_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/fr3_husky/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      FR3HuskyControllerQT
# )
# # ===================== FR3 XLS =====================
# mrs_add_controller(fr3_xls_controller
#   src/fr3_xls/controller.cpp
#   src/fr3_xls/robot_data.cpp
# )

# target_link_libraries(fr3_xls_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(fr3_xls_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/fr3_xls/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      FR3XLSControllerQT
# )
# # ===================== FR3 PCV =====================
# mrs_add_controller(fr3_pcv_controller
#   src/fr3_pcv/controller.cpp
#   src/fr3_pcv/robot_data.cpp
# )

# target_link_libraries(fr3_pcv_controller PUBLIC
#   dyros_robot_controller::RobotData
#   dyros_robot_controller::RobotController
# )

# ament_target_dependencies(fr3_pcv_controller PUBLIC 
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

# install(PROGRAMS    ${PROJECT_NAME}/fr3_pcv/ros2_qt_gui.py
#         DESTINATION lib/${PROJECT_NAME}
#         RENAME      FR3PCVControllerQT
# )
# # ===============================================



install(DIRECTORY robot/    DESTINATION share/${PROJECT_NAME}/robot)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(FILES plugins.xml   DESTINATION share/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(mujoco_ros_sim plugins.xml)
            
ament_python_install_package(${PROJECT_NAME})
ament_package()