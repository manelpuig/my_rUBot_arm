cmake_minimum_required(VERSION 3.8)
project(mecanum_5dof_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(
    FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()