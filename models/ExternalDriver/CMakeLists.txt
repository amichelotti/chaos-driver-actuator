cmake_minimum_required(VERSION 2.8)
project(chaos_driver_actuator_external)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)

FILE(GLOB src *.cpp)

IF(BUILD_FORCE_STATIC)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${src})
ELSE()
ADD_LIBRARY(${PROJECT_NAME} SHARED ${src})
ENDIF()
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_actuators chaos_driver_actuator chaos_cutoolkit chaos_common)
FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/actuator/models/ExternalDriver/)

INSTALL_TARGETS(/lib ${PROJECT_NAME} )





