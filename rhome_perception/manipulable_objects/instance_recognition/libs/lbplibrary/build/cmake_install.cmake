# Install script for directory: /home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/build/liblbp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblbp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/package_lbp" TYPE FILE FILES
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/LBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/bglbp/BGLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/cslbp/CSLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/csldp/CSLDP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/cssiltp/CSSILTP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/elbp/ELBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/oclbp/OCLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/olbp/OLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/scslbp/SCSLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/siltp/SILTP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/varlbp/VARLBP.h"
    "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/package_lbp/xcslbp/XCSLBP.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xappx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/build/lbp")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp"
         OLD_RPATH "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/build:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/lbp")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lmartinez/ros2_ws/src/rhome_perception/manipulable_objects/instance_recognition/libs/lbplibrary/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
