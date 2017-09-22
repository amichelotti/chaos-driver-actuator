cmake_minimum_required(VERSION 2.8)
project(chaos_driver_actuator_technosoft)


message(STATUS "Technosoft driver ${DRIVERS_LIB}")

FILE(GLOB src *.cpp)

ADD_LIBRARY(${PROJECT_NAME} ${src})

ADD_EXECUTABLE(test_technosoft_cu test/main.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_actuators common_actuators_technosoft)
TARGET_LINK_LIBRARIES(test_technosoft_cu ${PROJECT_NAME} chaos_driver_actuator common_actuators common_actuators_technosoft chaos_cutoolkit chaos_common ${FrameworkLib})
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
FILE(GLOB src *.h)

installHeaders(.)
INSTALL_TARGETS(/lib ${PROJECT_NAME} )
INSTALL_TARGETS(/bin test_technosoft_cu)





