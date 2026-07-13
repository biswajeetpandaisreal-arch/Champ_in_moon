cmake_minimum_required(VERSION 3.5)
project(champ_moon_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/maps)
  install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(PROGRAMS
    scripts/breadcrumb_node.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
