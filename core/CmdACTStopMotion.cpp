/*
CmdACTStopMotion.cpp
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
#include "CmdACTStopMotion.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;

BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTStopMotion,CMD_ACT_STOPMOTION_ALIAS,
		"Stop the Motion of the Actuator, if any",
		"63768ac0-11dc-11e6-8629-233988a40683")
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
//uint8_t own::CmdACTStopMotion::implementedHandler(){
//	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
// empty set handler
void own::CmdACTStopMotion::setHandler(c_data::CDataWrapper *data) {
	int err=0;
	AbstractActuatorCommand::setHandler(data);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"user_command_failed", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	SCLDBG_ << "Stop Motion " ;

	//setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, *p_setTimeout);
        if (actuator_drv != 0)
		SCLDBG_ << "driver pointer not null "<< actuator_drv << " axID "<< *axID;
	if((err = actuator_drv->stopMotion(*axID)) != 0) {

		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("error stopping motion")) );
		setStateVariableSeverity(StateVariableTypeAlarmCU,"user_command_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	//actuator_drv->accessor->base_opcode_priority=100;
	*p_stopCommandInExecution=true;
	BC_NORMAL_RUNNING_PROPERTY;
}
// empty acquire handler
void own::CmdACTStopMotion::acquireHandler() {

	//acquire the current readout
	AbstractActuatorCommand::acquireHandler();
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}
// empty correlation handler
void own::CmdACTStopMotion::ccHandler() {
	//int err=0;
	//double position;
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();

	//the command is endedn because we have reached the affinitut delta set
	SCLDBG_ << "cc handler Stop motion " << ((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION) ;
	//DPRINT(" :   %x %x",*o_status_id, ::common::actuators::ACTUATOR_INMOTION);

	if (((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION)==0)
	{
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,CHAOS_FORMAT("motion stopped",));

		BC_END_RUNNING_PROPERTY;
		return;
	}
	if (*o_alarms) {
		SCLERR_ << "We got alarms on actuator so we end the command";

		//metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing stop motion command: operation failed because of alarms detection")));
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"performing stop motion command: operation failed because of alarms detection");
		//setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
	}
}
// empty timeout handler
bool own::CmdACTStopMotion::timeoutHandler() {

	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"time out for stop motion command");

	//actuator_drv->accessor->base_opcode_priority=50;
	if ((*o_status_id && ::common::actuators::ACTUATOR_INMOTION)==0)
	{
		//metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("timeout for stop command. Motorwas stopped. Elapsed time %1%" , % elapsed_msec));
		SCLDBG_ << "[metric] Motor Stopped on timeout in " << elapsed_msec << " milliseconds";
		//the command is endedn because we have reached the affinitut delta set

		BC_END_RUNNING_PROPERTY;
	}else {
		SCLDBG_ << "[metric] Motor not stopped before timeout of " << elapsed_msec << " milliseconds";

		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Motor not stopped before timeout");
		BC_FAULT_RUNNING_PROPERTY;
	}
	return false;	
}
