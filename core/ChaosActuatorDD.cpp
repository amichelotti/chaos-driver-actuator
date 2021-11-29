/*
 *	Actuator base
 *	!CHAOS
 *	Created by Alessandro D'Uffizi
 *
 *    	Copyright 2013 INFN, National Institute of Nuclear Physics
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
#include "ChaosActuatorDD.h"

#include "driver/actuator/core/ChaosActuatorDD.h"
#include <boost/regex.hpp>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <string>

#define ACLAPP LAPP_ << "[ActuatorDD] "
#define ACDBG LDBG_ << "[ActuatorDD] "
#define ACERR LERR_ << "[ActuatorDD] "

using namespace chaos::driver::actuator;
// default constructor definition
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(chaos::driver::actuator,
                                             ChaosActuatorDD) {
  motor = NULL;
}

// default destructor
ChaosActuatorDD::~ChaosActuatorDD() {}

void ChaosActuatorDD::driverDeinit() {
  if (motor) {
    delete motor;
  }
  motor = NULL;
}

cu_driver::MsgManagmentResultType::MsgManagmentResult
ChaosActuatorDD::execOpcode(cu_driver::DrvMsgPtr cmd) {
 // boost::mutex::scoped_lock lock(io_mux);

  cu_driver::MsgManagmentResultType::MsgManagmentResult result =
      cu_driver::MsgManagmentResultType::MMR_EXECUTED;
  
  return result;
}
int ChaosActuatorDD::setTimeout(int32_t axisID,uint64_t timeo_ms) {
    return 0;
    
}

int ChaosActuatorDD::getTimeout(int32_t axisID,uint64_t* timeo_ms) {
return 0;
}

int ChaosActuatorDD::getPosition(int32_t axisID,::common::actuators::AbstractActuator::readingTypes readingType,double *deltaPosition_mm) {
    return motor->getPosition(axisID,readingType,deltaPosition_mm);
}

int ChaosActuatorDD::resetAlarms(int32_t axisID,uint64_t alrm){
    return motor->resetAlarms(axisID,alrm);
    
}

int ChaosActuatorDD::getAlarms(int32_t axisID,uint64_t*alrm,std::string& desc){
    return motor->getAlarms(axisID,alrm,desc);
}

int ChaosActuatorDD::moveRelative(int32_t axisID,double mm) {
    return motor->moveRelative(axisID,mm);  
}
int ChaosActuatorDD::moveAbsolute(int32_t axisID,double mm) {
    return motor->moveAbsolute(axisID,mm);
 
    
}
int ChaosActuatorDD::configAxis(int axis,void *ini){
  return motor->configAxis(axis,ini);
}

int ChaosActuatorDD::stopMotion(int32_t axisID){
    return motor->stopMotion(axisID);
    //DPRINT("ALEDEBUG STOP MOTION IN INTERFACE\n");
}
int ChaosActuatorDD::hardreset(int32_t axisID,bool mode){
    return motor->hardreset(axisID,mode);
}
int ChaosActuatorDD::homing(int32_t axisID,homingType mode){
    return motor->homing(axisID,mode);
}
int ChaosActuatorDD::soft_homing(int32_t axisID,double positionToSet){
    return motor->soft_homing(axisID,positionToSet);
}

int ChaosActuatorDD::poweron(int32_t axisID,int on){
    return motor->poweron(axisID,on);
}

int ChaosActuatorDD::getState(int32_t axisID,int* state,std::string& desc){
    return motor->getState(axisID,state,desc);

}


int ChaosActuatorDD::listParameters(std::string& dataset){
    return motor->listParameters(dataset);
}

int ChaosActuatorDD::getSWVersion(int32_t axisID,std::string& ver){
    return motor->getSWVersion(axisID,ver);
}

int ChaosActuatorDD::getHWVersion(int32_t axisID,std::string&ver){
    return motor->getHWVersion(axisID,ver);

}


uint64_t ChaosActuatorDD::getFeatures() {
  return motor->getFeatures();
    
}
int ChaosActuatorDD::setParameter(int32_t axisID,const std::string parName,const std::string value) {
  return motor->setParameter(axisID,parName,value);
}


int ChaosActuatorDD::getParameter(int axisID,std::string parName,std::string& resultString) {
  return motor->getParameter(axisID,parName,resultString);
}
int ChaosActuatorDD::initACT(int axis,void*ini){
  return motor->initACT(axis,ini);


}
    
int ChaosActuatorDD::deinitACT(int axisID){
  return motor->deinitACT(axisID);

}