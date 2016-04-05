//
//  ChaosActuatorInterface.h
//  Generic Actuator Interface to be used into CU
//
//  Created by Alessandro D'Uffizi  on 02/16/16.
//  Copyright (c) 2013 infn. All rights reserved.
//

#ifndef __ChaosActuatorInterface__
#define __ChaosActuatorInterface__

#include <iostream>
#include <chaos/cu_toolkit/driver_manager/driver/DriverTypes.h>
#include <chaos/cu_toolkit/driver_manager/driver/DriverAccessor.h>
#include <common/actuators/core/AbstractActuator.h>

#include <stdint.h>
namespace chaos_driver=::chaos::cu::driver_manager::driver;
namespace chaos {
    namespace driver {
#define MAX_STR_SIZE 256
        namespace actuator {
            typedef enum {
                OP_INIT = chaos_driver::OpcodeType::OP_USER, // init low level driver
                OP_DEINIT, // deinit low level driver
                OP_SET_TIMEOUT,
                OP_GET_TIMEOUT,
                OP_SET_SPEED, // set speed
                OP_SET_ACCELERATION, // set acceleration
                
                OP_SET_ADDITIVE, //
                OP_SET_REFERENCE, //
                OP_SET_MOVEMENT, //
                OP_GET_POSITION,
                
                OP_RESET_ALARMS,
                OP_GET_ALARMS,
                OP_MOVE_RELATIVE_MM,
                OP_MOVE_ABSOLUTE_MM,
                OP_STOP_MOTION,
                OP_HOMING,
                OP_POWERON,
                OP_GET_STATE,
                OP_GET_SWVERSION,
                OP_GET_HWVERSION,
                OP_GET_ALARM_DESC,
                OP_GET_FEATURE
            } ChaosActuatorOpcode;
            
            typedef struct {
                double fvalue0;
                double fvalue1;

                int ivalue;
                uint32_t timeout;
                uint64_t alarm_mask;
                
            } actuator_iparams_t;
            
            typedef struct {
                double fvalue0;
                double fvalue1;
                int ivalue;
                int result;
                uint64_t alarm_mask;
                char str[MAX_STR_SIZE];
            } actuator_oparams_t;
            
            // wrapper interface
            class ChaosActuatorInterface:public ::common::actuators::AbstractActuator {
            protected:

                chaos_driver::DrvMsg message;
                
            public:
	    ChaosActuatorInterface(chaos_driver::DriverAccessor*_accessor):accessor(_accessor){};
		chaos_driver::DriverAccessor* accessor;
                
                     /**
                 @brief initialize and poweron the power supply
                 @return 0 if success
                 */
                int init(void*);
                /**
                 @brief de-initialize the power supply and close the communication
                 @return 0 if success
                 */
                int  deinit();
                int setTimeout(uint64_t timeo_ms);   
                int getTimeout(uint64_t* timeo_ms);  
                
                int setSpeed(double speed_mm_per_sec);
                int setAcceleration(double acceleration_mm_per_sec2);
                void setAdditive(bool isAdditive);
                int setReferenceBase(int32_t referenceBase);
                int setMovement(int32_t movement);
                 /**
                 @brief return the position of the step motor in mm starting from the
                  * home position)
                 @param readingType the position reading method to be used to
                 @return 0 if success or an error code
                 */
                int getPosition(readingTypes readingType,double *deltaPosition_mm);
                
                int resetAlarms(uint64_t alrm);
                
                
                /**
                 @brief returns the bitfield of implemented alarms
                 @param alarm 64bit bitfield containing the implemented alarms
                 @return 0 if success or an error code
                 */
                
                int getAlarms(uint64_t*alrm);
                
                
                int moveRelativeMillimeters(double mm);
                int moveAbsoluteMillimeters(double mm);
                  /**
                 @brief stop the motion of the actuator (if is in movement)
                 @return 0 if success or an error code
                 */
                int stopMotion();
                
                /**
                 @brief put back the step motor to the home position)
                 @return 0 if success or an error code
                 */
                int homing(homingType mode);
                

                int poweron(uint32_t timeo_ms=ACTUATORS_DEFAULT_TIMEOUT);
                /**
                 @brief gets the power supply state
                 @param state returns a bit field of PowerSupplyStates
                 @param desc return a string description
                 @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
                 @return 0 if success or an error code
                 */
                
                int getState(int* state,std::string& desc);
           
                /**
                 @brief returns the SW/FW version of the driver/FW
                 @param version returning string
                 @return 0 if success or an error code
                 */
                int getSWVersion(std::string& version);
                
                /**
                 @brief returns the HW version of the powersupply
                 @param version returning string
                 @return 0 if success or an error code
                 */
                int getHWVersion(std::string& version);
            
                /**
                 @brief return a bitfield of capabilities of the actuator
                 @return the a bit field of capability
                 */
                uint64_t getFeatures() ;
              
                
                
                
                
                
               
                
                
                
                
                
            };
        }
    }
}

namespace chaos_actuator_dd = chaos::driver::actuator;

#endif
