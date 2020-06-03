cmake_minimum_required(VERSION 2.8)
project(chaos_driver_actuator)
find_package(chaos MODULE REQUIRED)
if(WIN32)
include_directories(${ALL_WINDOWS_INCLUDE_DIR})
else()
include_directories(../ ${chaos_INCLUDE_DIRS})
endif(WIN32)

FILE(GLOB core_src core/*.cpp)
SET(DRIVERS_LIB )
add_subdirectory(models)

IF(BUILD_FORCE_STATIC)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})
ELSE()
ADD_LIBRARY(${PROJECT_NAME} SHARED ${core_src})
ENDIF()

if(WIN32)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ALL_WINDOWS_LIBPATH} "-LIBPATH:C:/Users/aduffizi/source/repos/chaos_bundle/INSTALLPATH/lib/" chaos_common.lib chaos_cutoolkit common_debug)
else()
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  chaos_common common_debug  chaos_cutoolkit ${FrameworkLib})
endif()


FILE(GLOB files "core/*.h")
INSTALL(FILES ${files} DESTINATION include/driver/actuator/core)


INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

