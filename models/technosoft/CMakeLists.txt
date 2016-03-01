cmake_minimum_required(VERSION 2.6)
project(chaos_driver_actuator_technosoft)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} common_actuators PARENT_SCOPE)

message(STATUS "Technosoft driver ${DRIVERS_LIB}")
add_definitions(-DCHAOS -DDEBUG)
FILE(GLOB src *.cpp)

IF(BUILD_FORCE_STATIC)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${src})
ELSE()
ADD_LIBRARY(${PROJECT_NAME} SHARED ${src})
ENDIF()
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_driver_actuator common_actuators)
INSTALL_TARGETS(/lib ${PROJECT_NAME} )





