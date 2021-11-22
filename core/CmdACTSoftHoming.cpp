/*
CmdACTSoftHoming.cpp
!CHAOS
Created by A. D'Uffizi
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
#include "CmdACTSoftHoming.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTSoftHoming) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTSoftHoming) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTSoftHoming) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTSoftHoming,CMD_ACT_SOFT_HOMING_ALIAS,
		"Manually set the actuator current position",
		"f096e257-1790-11e6-8845-1f6ac6d4e985")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_SOFT_HOMING_POS,"current position",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

void own::CmdACTSoftHoming::setHandler(c_data::CDataWrapper *data) 
{
	AbstractActuatorCommand::setHandler(data);

	int err = 0;
	//double max_homing_type=0,min_homing_type=0;
	double currentPosition;
	double HomePositionToSet;
	uint64_t computed_timeout;
	*p_stopCommandInExecution=false;
    
	setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	int32_t *lock=getAttributeCache()->getRWPtr<int32_t>(DOMAIN_INPUT, "Lock");
    if  ((lock != NULL) && (*lock > 0) )
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
    if(!data ||
			!data->hasKey(CMD_ACT_SOFT_HOMING_POS)) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"current position not specified ");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	if(!data->isDoubleValue(CMD_ACT_SOFT_HOMING_POS) && (!data->isInt32Value(CMD_ACT_SOFT_HOMING_POS))) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Homing  parameter is not a double data type");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
    currentPosition = data->getDoubleValue(CMD_ACT_SOFT_HOMING_POS);
    
    HomePositionToSet= currentPosition;

    	SCLDBG_ << "Launching soft homing command with position as home " << HomePositionToSet;
	if(*o_stby==0){
		// we are in standby only the SP is set
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"cannot perform soft homing because in PowerOff");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	*o_lasthoming = 0;
	*o_kindofhome = 0;
	if((err = actuator_drv->soft_homing(*axID,HomePositionToSet)) < 0)
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("axis %1% cannot perform soft homing: driver err: %2%") %*axID %err));
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdACTSoftHoming::acquireHandler()
{
	AbstractActuatorCommand::acquireHandler();
}
void own::CmdACTSoftHoming::ccHandler()
{
	*o_lasthoming = chaos::common::utility::TimingUtil::getTimeStamp();
	*o_kindofhome = 2;
	
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,"Soft-Homing completed");
    //*i_position=*o;
 	getAttributeCache()->setInputDomainAsChanged();
	getAttributeCache()->setOutputDomainAsChanged();
	BC_END_RUNNING_PROPERTY;
}
bool own::CmdACTSoftHoming::timeoutHandler()
{
	BC_END_RUNNING_PROPERTY;
	return false;
}