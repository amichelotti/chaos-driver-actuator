cmake_minimum_required(VERSION 2.9)
project(chaos_driver_pollux)


message(STATUS "Actuator Pollux driver ${DRIVERS_LIB}")

FILE(GLOB src *.cpp)

ADD_LIBRARY(${PROJECT_NAME} ${src}  )

#ADD_EXECUTABLE(test_actuator_sim_cu test/main.cpp)
 TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_actuators )
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
FILE(GLOB src *.h)

FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/actuator/models/polluxVenus2/)



INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
#INSTALL_TARGETS(/bin test_technosoft_cu)





