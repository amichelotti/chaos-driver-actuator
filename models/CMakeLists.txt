cmake_minimum_required(VERSION 2.8)
project(actuator_models)
IF("${CMAKE_C_COMPILER}" MATCHES "i686-nptl-linux-gnu-gcc") 
  IF( NOT CHAOS_STATIC)
    MESSAGE(STATUS "Enabling compilation of technosoft")
    add_subdirectory(technosoft)
  ELSE()
    MESSAGE(STATUS "Disabling compilation of technosoft, because static")
  ENDIF()
ELSE()
  MESSAGE(STATUS "Disabling compilation of technosoft, wrong cross compiler")
ENDIF()
add_subdirectory(actuatorSim)



SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)
