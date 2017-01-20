/*
CmdACTPoweron.cpp
!CHAOS
Created by CUGenerator

Copyright 2013 INFN, National Institute of Nuclear Physics
Licensed under the Apache License, Version 2.0 (the "License")
you may not use this file except in compliance with the License.
      You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

 */
#include "CmdACTPoweron.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;

BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTPoweron,CMD_ACT_POWERON_ALIAS,
		"Turn on the power of the actuator",
		"29d39d4f-0311-4774-b0b3-8caa7862193c")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_POWERON_VALUE,"on state",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)

BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
//uint8_t own::CmdACTPoweron::implementedHandler(){
//	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
// empty set handler
void own::CmdACTPoweron::setHandler(c_data::CDataWrapper *data) {
	int err;
	AbstractActuatorCommand::setHandler(data);
	setWorkState(true);

	if(!data ||!data->hasKey(CMD_ACT_POWERON_VALUE)) {
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"parameter  0/1 must be specified" );
			BC_FAULT_RUNNING_PROPERTY;
			return;
	}

<<<<<<< HEAD
	
SCLDBG_   << "Launching poweron in set handler power on in axid "<< *axID << " value " << onState;
	if((err = actuator_drv->stopMotion(*axID)) != 0) {
            SCLDBG_ << "Error while stopping motion of the actuator ";
}
=======
	onState = data->getInt32Value(CMD_ACT_POWERON_VALUE);

	setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	SCLDBG_   << "Launching poweron in set handler power on in axid "<< *axID << " value " << onState;
>>>>>>> ad4d5c4adecb05e62c91e18d11dde09204322b03
	if((err = actuator_drv->poweron(*axID,onState)) != 0) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("axis %1% cannot perform set state (poweron) to ",%*axID %onState));
		setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
		return;

	}
	BC_NORMAL_RUNNING_PROPERTY;
}
// empty acquire handler
void own::CmdACTPoweron::acquireHandler() {
	AbstractActuatorCommand::acquireHandler();

}
// empty correlation handler
void own::CmdACTPoweron::ccHandler() {
	if((((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)?1:0)==onState){
		BC_END_RUNNING_PROPERTY;
	}
}
// empty timeout handler
bool own::CmdACTPoweron::timeoutHandler() {
	AbstractActuatorCommand::acquireHandler();

	if((((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)?1:0)==onState){
			BC_END_RUNNING_PROPERTY;
			return false;
	}
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

	BC_END_RUNNING_PROPERTY;
	return false;

}
