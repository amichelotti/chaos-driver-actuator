cmake_minimum_required(VERSION 2.8)
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
INSTALL(FILES ${src} DESTINATION include/driver/actuators/models/technosoft)
ADD_EXECUTABLE(test_technosoft_cu test/main.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_driver_actuator common_actuators common_actuators_technosoft)
TARGET_LINK_LIBRARIES(test_technosoft_cu ${PROJECT_NAME} chaos_driver_actuator common_actuators common_actuators_technosoft chaos_cutoolkit chaos_common ${FrameworkLib})
FILE(GLOB src *.h)

INSTALL_TARGETS(/lib ${PROJECT_NAME} )
INSTALL_TARGETS(/bin test_technosoft_cu)





