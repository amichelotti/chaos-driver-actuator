/*
 *      Actuator Device Driver External
 *	!CHAOS
 *	Created by Andrea Michelotti
 *
 *    	Copyright 2020 INFN, National Institute of Nuclear Physics
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
#ifndef __driver_ChaosActuatorExternalDD_h__
#define __driver_ChaosActuatorExternalDD_h__

// include your class/functions headers here

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractServerRemoteIODriver.h>
#include <chaos/cu_toolkit/driver_manager/driver/OpcodeDriverWrapper.h>

#include <common/actuators/core/AbstractActuator.h>
// this need to be out the nasmespace
DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(ChaosActuatorExternalDD)
namespace cu_driver = chaos::cu::driver_manager::driver;

namespace chaos {
namespace driver {
namespace actuator {

/*
 driver definition
 */
class ChaosActuatorExternalDD : ADD_CU_DRIVER_PLUGIN_SUPERCLASS,
                                public ::common::actuators::AbstractActuator {

protected:
  // chaos::cu::driver_manager::driver::AbstractClientRemoteIODriver
  boost::mutex io_mux;
  chaos::cu::driver_manager::driver::AbstractClientRemoteIODriver client;
  // chaos::cu::driver_manager::driver::OpcodeDriverWrapper<ChaosActuatorExternalDD,
  // chaos::cu::driver_manager::driver::AbstractServerRemoteIODriver> client;
public:
  ChaosActuatorExternalDD();
  ~ChaosActuatorExternalDD();
  //! Execute a command
  cu_driver::MsgManagmentResultType::MsgManagmentResult
  execOpcode(cu_driver::DrvMsgPtr cmd);
  int asyncMessageReceived(chaos::common::data::CDWUniquePtr message);
  void driverDeinit() throw(chaos::CException);
  void driverInit(const char *) throw(chaos::CException);

  void driverInit(const chaos::common::data::CDataWrapper
                      &init_parameter) throw(chaos::CException);
  int initACT(void *);
  /**
   @brief de-initialize the power supply and close the communication
   @return 0 if success
   */
  uint64_t setGeneralInterfaceTimeout(uint64_t timeo_ms);
  int configAxis(void *);
  int deinitACT(int32_t axisID);
  int setTimeout(int32_t axisID, uint64_t timeo_ms);
  int getTimeout(int32_t axisID, uint64_t *timeo_ms);

  // int setSpeed(double speed_mm_per_sec);
  // int setAcceleration(double acceleration_mm_per_sec2);
  uint64_t getGeneralInterfaceTimeout();
  int setAdditive(bool isAdditive);
  int setReferenceBase(int32_t referenceBase);
  // int setMovement(int32_t movement);
  int sethighSpeedHoming(double speed_mm_per_sec);
  int setlowSpeedHoming(double speed_mm_per_sec);
  int setAccelerationHoming(double acceleration_mm_per_sec2);
  int setAdditiveHoming(bool isAdditive);
  int setReferenceBaseHoming(int32_t referenceBase);
  int setMovementHoming(int32_t movement);
  /**
      @brief return the value of the parameter specified in parName, currently
     inside the driver

                  @param parName the name of parameter. This parameter must be
     one of the parameters declared by the driver in sendDataset
                  @param  axisID  the axis Identifier of the motor
                  @param  resultString  contains a string with the value of
     parameter. Correct conversion from string is demanded to the CU
                  @return 0 if success or an error code
                  */
  int getParameter(int axisID, std::string parName, std::string &resultString);

  int setParameter(int32_t axisID, std::string parName, std::string value);
  /**
  @brief return the position of the step motor in mm starting from the
   * home position)
  @param readingType the position reading method to be used to
  @return 0 if success or an error code
  */

  int getPosition(int32_t axisID, readingTypes readingType,
                  double *deltaPosition_mm);

  int resetAlarms(int32_t axisID, uint64_t alrm);
  int hardreset(int32_t axisID, bool mode);

  /**
   @brief returns the bitfield of implemented alarms
   @param alarm 64bit bitfield containing the implemented alarms
   @return 0 if success or an error code
   */

  int getAlarms(int32_t axisID, uint64_t *alrm, std::string &desc);

  int moveRelative(int32_t axisID, double mm);
  int moveAbsolute(int32_t axisID, double mm);
  /**
 @brief stop the motion of the actuator (if is in movement)
 @return 0 if success or an error code
 */
  int stopMotion(int32_t axisID);

  /**
   @brief put back the step motor to the home position)
   */
  int homing(int32_t axisID, homingType mode);
  // int setTrapezoidalProfile(double, double, int, short int, short int);

  int poweron(int32_t axisID, int on);
  /**
   @brief gets the power supply state
   @param state returns a bit field of PowerSupplyStates
   @param desc return a string description
   @param timeo_ms timeout in ms for the completion of the operation (0 wait
   indefinitively)
   @return 0 if success or an error code
   */

  int getState(int32_t axisID, int *state, std::string &desc);

  /**
   @brief returns the SW/FW version of the driver/FW
   @param version returning string
   @return 0 if success or an error code
   */
  int getSWVersion(int32_t axisID, std::string &version);

  /**
   @brief returns the HW version of the powersupply
   @param version returning string
   @return 0 if success or an error code
   */
  int getHWVersion(int32_t axisID, std::string &version);
  int listParameters(std::string &dataset);

  /**
   @brief return a bitfield of capabilities of the actuator
   @return the a bit field of capability
   */
  uint64_t getFeatures();
  int setTrapezoidalProfile(double, double, bool, int32_t, int32_t);
  int setMaxSpeed(double);
};
} // namespace actuator
} // namespace driver
} // namespace chaos

#endif
