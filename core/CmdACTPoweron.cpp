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


void own::CmdACTPoweron::setHandler(c_data::CDataWrapper *data) {
	int err;
	AbstractActuatorCommand::setHandler(data);
 	i_stby=getAttributeCache()->getRWPtr<bool>(DOMAIN_INPUT, "powerOn");


	setWorkState(true);

	if(!data ||!data->hasKey(CMD_ACT_POWERON_VALUE)) {
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"parameter  0/1 must be specified" );
			BC_FAULT_RUNNING_PROPERTY;
			return;
	}


	onState = data->getInt32Value(CMD_ACT_POWERON_VALUE);
	SCLDBG_ << "ALEDEBUG received poweron value " <<onState ;

	setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);

   	uint64_t computed_timeout=std::max((uint64_t)5000000,(uint64_t)*p_setTimeout);
 	SCLDBG_ << "Calculated timeout is = " << computed_timeout;
        setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);


	SCLDBG_   << "Launching poweron in set handler power on in axid "<< *axID << " value " << onState;
	if((err = actuator_drv->poweron(*axID,onState)) != 0) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("axis %1% cannot perform set state (poweron) to ",%*axID %onState));
		setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
		return;

	}
	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdACTPoweron::endHandler() 
{
		SCLDBG_ << "removing busy flag";
    setWorkState(false);
    AbstractActuatorCommand::endHandler();
}


// empty acquire handler
void own::CmdACTPoweron::acquireHandler() {
	AbstractActuatorCommand::acquireHandler();

}
// empty correlation handler
void own::CmdACTPoweron::ccHandler() {
	if((((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)?1:0)==onState){
	         SCLDBG_ << " cchandler power on command on axisID " << *axID << " onstate "<< onState;

		*o_stby=(onState==1);
		*i_stby=(onState==1);
 		getAttributeCache()->setInputDomainAsChanged();

		BC_END_RUNNING_PROPERTY;
	}
}
// empty timeout handler
bool own::CmdACTPoweron::timeoutHandler() {
	AbstractActuatorCommand::acquireHandler();

	if((((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)?1:0)==onState){
	SCLDBG_ << "power on ok in timeout command on axisID " << *axID;
			*o_stby=(onState==1);
			*i_stby=(onState==1);
 		getAttributeCache()->setInputDomainAsChanged();
			BC_END_RUNNING_PROPERTY;
			return false;
	}
	setStateVariableSeverity(StateVariableTypeAlarmCU,"powerOn_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

	BC_END_RUNNING_PROPERTY;
	return false;

}
