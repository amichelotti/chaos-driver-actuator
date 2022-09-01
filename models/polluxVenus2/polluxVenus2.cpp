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
#include "polluxVenus2.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

#define INITDRIVER_DEF
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(polluxVenus2, 1.0.0, chaos::driver::actuator::polluxVenus2)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::polluxVenus2, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

// register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(polluxVenus2)
CLOSE_REGISTER_PLUGIN

#ifndef ACTAPP
#define ACTAPP LAPP_ << "[polluxVenus2] "
#define ACTERR LERR_ << "[polluxVenus2] "
#define ACTDBG LDBG_ << "[polluxVenus2] "
#endif
namespace chaos {
namespace driver {
namespace actuator {
// default constructor definition
polluxVenus2::polluxVenus2() {
}

// default descrutcor
polluxVenus2::~polluxVenus2() {
}
void polluxVenus2::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException) {
  int ret = -1;
  ACTDBG << "Init driver initialization with json " << json.getJSONString().c_str();
  serial = ::common::serial::SerialChannelFactory::getChannel(json);

  if (serial.get() == NULL) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(1, "Bad parameters for polluxVenus2", "polluxVenus2::driverInit");
  } else {
    ACTDBG << "Init Done" << std::endl;
  }

  if ((ret = serial->init()) != 0) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(-2, "Cannot connect", "polluxVenus2::driverInit");
  }
}

void polluxVenus2::driverInit(const char* initParameter) throw(chaos::CException) {
  // rett= motor->getPosition((::common::actuators::AbstractActuator::readingTypes)1,&mmpos);
}

int polluxVenus2::setParameter(int axisID, std::string parName, std::string value) {
  return 0;
}
int polluxVenus2::moveRelative(int axisID, double mm) {
  return 0;
}
int polluxVenus2::getParameter(int axisID, std::string parName, std::string& resultString) {
  return 0;
}
int polluxVenus2::getPosition(int axisID, readingTypes mode, double* deltaPosition_mm) {
  return 0;
}
int polluxVenus2::initACT(int axisID, void* par) {
  return 0;
}
int polluxVenus2::configAxis(int axisID, void* initialization_string) {
  return 0;
}
int polluxVenus2::deinitACT(int axisID) {
  return 0;
}
int polluxVenus2::hardreset(int axisID, bool mode) {
  return 0;
}

int polluxVenus2::getSWVersion(int axisID, std::string& version) {
  return 0;
}
int polluxVenus2::getHWVersion(int axisID, std::string& version) {
  return 0;
}
int polluxVenus2::listParameters(std::string& dataset) {
  return 0;
}
int polluxVenus2::stopMotion(int axisID) {
  return 0;
}
int polluxVenus2::homing(int axisID, homingType mode) {
  return 0;
}
int polluxVenus2::soft_homing(int axisID, double positionToSet) {
  return 0;
}
int polluxVenus2::getState(int axisID, int* state, std::string& desc) {
  return 0;
}
int polluxVenus2::getAlarms(int axisID, uint64_t* alrm, std::string& desc) {
  return 0;
}
int polluxVenus2::resetAlarms(int axisID, uint64_t alrm) {
  return 0;
}
int polluxVenus2::poweron(int axisID, int on) {
  return 0;
}
uint64_t polluxVenus2::getFeatures() {
  return 0;
}
int polluxVenus2::moveAbsolute(int axisID, double mm) {
  return 0;
}
}  // namespace actuator
}  // namespace driver
}  // namespace chaos