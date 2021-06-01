# Install script for directory: /home/dhruv/final_ws/src/aws_robomaker_bookstore_world

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dhruv/final_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dhruv/final_ws/build/aws_robomaker_bookstore_world/catkin_generated/installspace/aws_robomaker_bookstore_world.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_bookstore_world/cmake" TYPE FILE FILES
    "/home/dhruv/final_ws/build/aws_robomaker_bookstore_world/catkin_generated/installspace/aws_robomaker_bookstore_worldConfig.cmake"
    "/home/dhruv/final_ws/build/aws_robomaker_bookstore_world/catkin_generated/installspace/aws_robomaker_bookstore_worldConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_bookstore_world" TYPE FILE FILES "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_bookstore_world" TYPE DIRECTORY FILES
    "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/launch"
    "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/models"
    "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/worlds"
    "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/maps"
    "/home/dhruv/final_ws/src/aws_robomaker_bookstore_world/routes"
    )
endif()

