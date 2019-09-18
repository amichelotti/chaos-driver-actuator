/*
 *	ChaosActuatorOpcodeLogic.h
 *
 *	!CHAOS
 *	Created by D'Uffizi.
 *
 *    	Copyright 25/07/2017 INFN, National Institute of Nuclear Physics
 *
 *    	Licensed under the Apache License, Version 2.0 (the "License");
 *    	you may not use this file except in compliance with the License.
 *    	You may obtain a copy of the License at
 *
 *    	http://www.apache.org/licenses/LICENSE-2.0
 *
 *    	Unless required by applicable law or agreed to in writing, software
 *    	distributed under the License is distributed on an "AS IS" BASIS,
 *    	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    	See the License for the specific language governing permissions and
 *    	limitations under the License.
 */
#ifndef ChaosActuatorOpcodeLogic_h
#define ChaosActuatorOpcodeLogic_h
#include <common/actuators/core/AbstractActuator.h>
#include <chaos/common/chaos_types.h>
#include <chaos/common/utility/LockableObject.h>
#include <chaos/cu_toolkit/driver_manager/driver/OpcodeExternalCommandMapper.h>
namespace chaos {
    namespace driver {
		namespace actuator {
			class ChaosActuatorOpcodeLogic: 
				public chaos::cu::driver_manager::driver::OpcodeExternalCommandMapper {
            protected:
         //       int sendInit(cu::driver_manager::driver::DrvMsgPtr cmd);
                
          //      int sendDeinit(cu::driver_manager::driver::DrvMsgPtr cmd);
				  /**
                    @brief return the value of the parameter specified in parName, currently inside the driver
                    @param parName the name of parameter. This parameter must be one of the parameters declared by the driver in sendDataset
                    @param  axisID  the axis Identifier of the motor
                    @param  resultString  contains a string with the value of parameter. Correct conversion from string is demanded to the CU
                    @return 0 if success or an error code
                                */
                int getParameter(cu::driver_manager::driver::DrvMsgPtr cmd,int axisID,std::string parName,std::string& resultString);
				int setParameter(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,std::string parName, std::string parValue);
				 /**
                 @brief return the position of the step motor starting from the
                  * home position)
                 @param readingType the position reading method to be used to
                 @return 0 if success or an error code
                 */
                int getPosition(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,::common::actuators::AbstractActuator::readingTypes readingType,double *deltaPosition);
				int resetAlarms(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,uint64_t alrm);
                int hardreset(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,bool mode);


                /**
                 @brief returns the bitfield of implemented alarms
                 @param alarm 64bit bitfield containing the implemented alarms
                 @return 0 if success or an error code
                 */
                int getAlarms(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,uint64_t*alrm, std::string& desc);
				int moveRelativeMillimeters(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,double delta);
                int moveAbsoluteMillimeters(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,double setPos);
                  /**
                 @brief stop the motion of the actuator (if is in movement)
                 @return 0 if success or an error code
                 */
                int stopMotion(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID);
				  /**
                 @brief put back the step motor to the home position)
                 */
                int homing(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,::common::actuators::AbstractActuator::homingType mode);
       
                int poweron(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,int on);
                /**
                 @brief gets the power supply state
                 @param state returns a bit field of PowerSupplyStates
                 @param desc return a string description
                 @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
                 @return 0 if success or an error code
                 */

                int getState(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,int* state,std::string& desc);

                /**
                 @brief returns the SW/FW version of the driver/FW
                 @param version returning string
                 @return 0 if success or an error code
                 */
                int getSWVersion(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,std::string& version);
				
                /**
                 @brief returns the HW version of the powersupply
                 @param version returning string
                 @return 0 if success or an error code
                 */
                int getHWVersion(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,std::string& version);
                int sendDataset(cu::driver_manager::driver::DrvMsgPtr cmd,std::string& dataset);

				 //int init(void*);
             
                int configAxis(cu::driver_manager::driver::DrvMsgPtr cmd,void*);
                int  deinit(cu::driver_manager::driver::DrvMsgPtr cmd, int32_t axisID);
                int setTimeout(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,uint64_t timeo_ms);
                int getTimeout(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,uint64_t* timeo_ms);

                //int setSpeed(double speed_mm_per_sec);
                //int setAcceleration(double acceleration_mm_per_sec2);
                int setAdditive(cu::driver_manager::driver::DrvMsgPtr cmd,bool isAdditive);
                int setReferenceBase(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t referenceBase);
                //int setMovement(int32_t movement);
                int sethighSpeedHoming(cu::driver_manager::driver::DrvMsgPtr cmd,double speed_mm_per_sec);
                int setlowSpeedHoming(cu::driver_manager::driver::DrvMsgPtr cmd,double speed_mm_per_sec);
                int setAccelerationHoming(cu::driver_manager::driver::DrvMsgPtr cmd,double acceleration_mm_per_sec2);
                int setAdditiveHoming(cu::driver_manager::driver::DrvMsgPtr cmd,bool isAdditive);
                int setReferenceBaseHoming(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t referenceBase);
				int setMovementHoming(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t movement);
				uint64_t getFeatures(cu::driver_manager::driver::DrvMsgPtr cmd) ;
                int setTrapezoidalProfile(cu::driver_manager::driver::DrvMsgPtr cmd,double, double, bool, int32_t, int32_t) ;
                int setMaxSpeed(cu::driver_manager::driver::DrvMsgPtr cmd,double speed);


				 public:
                ChaosActuatorOpcodeLogic(chaos::cu::driver_manager::driver::RemoteIODriverProtocol *_remote_driver);
                ~ChaosActuatorOpcodeLogic();
                void driverInit(const chaos::common::data::CDataWrapper& init_parameter) throw(chaos::CException);
                void driverDeinit() throw(chaos::CException);
                chaos::cu::driver_manager::driver::MsgManagmentResultType::MsgManagmentResult execOpcode(chaos::cu::driver_manager::driver::DrvMsgPtr cmd);
                int asyncMessageReceived(chaos::common::data::CDWUniquePtr message);




			};
		}



	}
}
#endif //ChaosActuatorOpcodeLogic_h
