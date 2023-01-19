/*
 *
 *	!CHAOS
 *	Created by Andrea Michelotti
 *
 *    	Copyright 2022 INFN, National Institute of Nuclear Physics
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
#ifndef __driver_polluxVenus2_h__
#define __driver_polluxVenus2_h__

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <common/serial/core/SerialChannelFactory.h>

#include <driver/actuator/core/ChaosActuatorDD.h>

// this need to be out the namespace

DEFINE_CU_DRIVER_DEFINITION_PROTOTYPE(polluxVenus2)
namespace cu_driver = chaos::cu::driver_manager::driver;

namespace chaos {
namespace driver {
namespace actuator {
#define POLLUX_TERMINATOR "\r\n"
/*
 driver definition
 */
class polluxVenus2 : public ChaosActuatorDD {
  void                                        driverInit(const char* initParameter) ;
  void                                        driverInit(const chaos::common::data::CDataWrapper& json) ;
  ::common::serial::AbstractSerialChannel_psh serial;
  uint64_t counter;
  ChaosMutex io_mux;
  std::map<int32_t,uint64_t> statusMap;
  
 protected:
  std::string getAnswer(int timeo_ms = 5000);
  int         sendCommand(int axis, const std::string& cmd, std::string& reply);
  int         sendCommand(int axis, const std::string& cmd);
  int         sendCommand(const std::string param, int axis, const std::string& cmd);

  int abortAll();
 public:
  polluxVenus2();
  ~polluxVenus2();

  int      setParameter(int axisID, std::string parName, std::string value);
  int      moveRelative(int axisID, double mm);
  int      getParameter(int axisID, std::string parName, std::string& resultString);
  int      getPosition(int axisID, readingTypes mode, double* deltaPosition_mm);  //***OK***
  int      initACT(int axisID, void*);
  int      configAxis(int axisID, void* initialization_string);
  int      deinitACT(int axisID);
  int      hardreset(int axisID, bool mode);
  int      getSWVersion(int axisID, std::string& version);
  int      getHWVersion(int axisID, std::string& version);
  int      listParameters(std::string& dataset);
  int      stopMotion(int axisID);
  int      homing(int axisID, homingType mode);
  int      soft_homing(int axisID, double positionToSet);
  int      getState(int axisID, int* state, std::string& desc);
  int      getAlarms(int axisID, uint64_t* alrm, std::string& desc);
  int      resetAlarms(int axisID, uint64_t alrm);
  int      poweron(int axisID, int on);
  uint64_t getFeatures();
  int      moveAbsolute(int axisID, double mm);
};
}  // namespace actuator
}  // namespace driver
}  // namespace chaos

#endif
