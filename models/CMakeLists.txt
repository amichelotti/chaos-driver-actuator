cmake_minimum_required(VERSION 2.8)
project(actuator_models)
IF("${CMAKE_C_COMPILER}" MATCHES "i686-nptl-linux-gnu-gcc") 
  IF( NOT CHAOS_STATIC)
    MESSAGE(STATUS "Enabling compilation of technosoft")
    FILE(REMOVE technosoft/UnitServerIgnore)
    add_subdirectory(technosoft)
  ELSE()
    MESSAGE(STATUS "Disabling compilation of technosoft, because static")
  ENDIF()
ELSE()
  FILE(WRITE technosoft/UnitServerIgnore "disabled ${PROJECT_NAME}")
  MESSAGE(STATUS "Disabling compilation of technosoft, wrong cross compiler")
ENDIF()
add_subdirectory(actuatorSim)
add_subdirectory(ExternalDriver)



SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)
