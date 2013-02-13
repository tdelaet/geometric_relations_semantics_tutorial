#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(geometric_semantics_component)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libgeometric_semantics_component-<target>.so
# and installs in the directory lib/orocos/geometric_semantics_component/
#
orocos_component(geometric_semantics_tutorial_orocos_components 
    geometric_semantics_component-tutorial-publisher.hpp geometric_semantics_component-tutorial-publisher.cpp
    geometric_semantics_component-tutorial-subscriber.hpp geometric_semantics_component-tutorial-subscriber.cpp
    ) # ...you may add multiple source files

# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/geometric_semantics_component/ directory
#
#orocos_install_headers( geometric_semantics_component-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libgeometric_semantics_component-plugin-<target>.so
# and installs in the directory lib/orocos/geometric_semantics_component/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(geometric_semantics_component-plugin geometric_semantics_component-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libgeometric_semantics_component-service-<target>.so
# and installs in the directory lib/orocos/geometric_semantics_component/plugins/
#
#orocos_service(geometric_semantics_component-service geometric_semantics_component-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit (recommended):
#
# Creates a typekit library libgeometric_semantics_component-types-<target>.so
# and installs in the directory lib/orocos/geometric_semantics_component/types/
#
#orocos_typegen_headers(geometric_semantics_component-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()

