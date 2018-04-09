cmake_minimum_required(VERSION 2.8)
project(chaos_driver_actuator)
FILE(GLOB core_src core/*.cpp)
SET(DRIVERS_LIB )
add_subdirectory(models)

IF(BUILD_FORCE_STATIC)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})
ELSE()
ADD_LIBRARY(${PROJECT_NAME} SHARED ${core_src})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common common_debug common_modbus modbus chaos_cutoolkit chaos_uitoolkit ${FrameworkLib})
installHeaders(core)
INSTALL_TARGETS(/lib ${PROJECT_NAME})


