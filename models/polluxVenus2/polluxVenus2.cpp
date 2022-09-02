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
std::string polluxVenus2::getAnswer(int timeo_ms){
  std::stringstream ss;
  char buf;
  int ret;
  while((ret=serial->read(&buf,1,timeo_ms))==1){
    ss<<buf;
    if(ss.str().find_last_of(POLLUX_TERMINATOR)!=std::string::npos){
      return ss.str();
    } else {
      //  ACTDBG << "receive buffer \"" << ss.str()<<"\" ret:"<<ret;

    }
  }
  return ss.str();
}
int polluxVenus2::abortAll(){
  char ctrlc=2;
  ACTDBG << "Abort ALL";

  return serial->write((void*)&ctrlc,1);
}
int polluxVenus2::sendCommand(int axis,const std::string&cmd){
    std::stringstream ss;
    ss<<axis<<" "<<cmd<<POLLUX_TERMINATOR;
    ACTDBG << "Sending \"" << ss.str()<<"\"";

   int ret=serial->write((void*)ss.str().c_str(),ss.str().size());
   return ret;
}

int polluxVenus2::sendCommand(int axis,const std::string&cmd,std::string&reply){
  reply="";
  int ret=sendCommand(axis,cmd);
  if(ret<0){
    getAnswer();
    return ret;
  }
  reply=getAnswer();
  return reply.size();
}

void polluxVenus2::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException) {
  int ret = -1;
  ACTDBG << "Init driver initialization with json " << json.getJSONString().c_str();
  if(!json.hasKey("driver_param")){
    throw chaos::CException(-1, "Missing driver_param key", "polluxVenus2::driverInit");

  }
  chaos::common::data::CDWUniquePtr channel=json.getCSDataValue("driver_param");
  serial = ::common::serial::SerialChannelFactory::getChannel(*channel.get());

  if (serial.get() == NULL) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(1, "Bad parameters for polluxVenus2:"+channel->getJSONString(), "polluxVenus2::driverInit");
  } else {
  }

  if ((ret = serial->init()) != 0) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(-2, "Cannot connect", "polluxVenus2::driverInit");
  }
  stopMotion(-1); // abort all movements (if any)
  
  

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
  std::string rets;
  int ret;
  if((ret=sendCommand(axisID,"npos",rets))<=0){
    return -1;
  } 
  *deltaPosition_mm=atof(rets.c_str());
  ACTDBG << "Axis " << axisID<<" position:"<<*deltaPosition_mm;

  return 0;
}
int polluxVenus2::initACT(int axisID, void* par) {
  sendCommand(axisID,"nclear"); // clear fifo parameter stack
  std::string version;
  int ret;
  if((ret=getHWVersion(axisID,version))<=0){
    ACTERR<<" cannot retrieve version ret:"<<ret;
    return ret;
  }
  ACTDBG << "Init Done, version:" << version<<std::endl;
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
  return  sendCommand(axisID,"nversion",version);

}
int polluxVenus2::getHWVersion(int axisID, std::string& version) {
  return sendCommand(axisID,"nidentify",version);

}
int polluxVenus2::listParameters(std::string& dataset) {
  return 0;
}
int polluxVenus2::stopMotion(int axisID) {
  if(axisID==-1){
    return abortAll();
  } 
  return sendCommand(axisID,"nabort");
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