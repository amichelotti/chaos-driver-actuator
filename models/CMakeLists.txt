cmake_minimum_required(VERSION 2.8)
project(actuator_models)
IF("${CMAKE_C_COMPILER}" MATCHES "i686-nptl-linux-gnu-gcc") 
  MESSAGE(STATUS "Enabling compilation of technosoft")
  add_subdirectory(technosoft)
  add_subdirectory(actuatorSim)
ELSE()
  MESSAGE(STATUS "Disabling compilation of technosoft")
ENDIF()



SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)
