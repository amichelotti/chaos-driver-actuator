/*
 *	CmdACTMoveRelative.cpp
 *	!CHAOS
 *	Created by Alessandro D'Uffizi.
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


#include "CmdACTMoveRelative.h"
#include <chaos/cu_toolkit/windowsCompliant.h>
#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

#define SCLAPP_ INFO_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "


namespace own =  driver::actuator;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;


BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTMoveRelative,CMD_ACT_MOVE_RELATIVE_ALIAS,
		"Move from current position to an offset (mm)",
		"0cf52f73-55eb-4e13-8712-3d54486040d8")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_MM_OFFSET, "offset in mm",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

// return the implemented handler    //************************** commentato *****************************
//uint8_t own::CmdACTMoveRelative::implementedHandler(){
//    return	AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}

void own::CmdACTMoveRelative::setHandler(c_data::CDataWrapper *data) {
	//chaos::common::data::RangeValueInfo position_sp_attr_info;   // ************* Commentato *************
	//chaos::common::data::RangeValueInfo attributeInfo;           // ************* Commentato *************
	AbstractActuatorCommand::setHandler(data);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	int32_t *lock=getAttributeCache()->getRWPtr<int32_t>(DOMAIN_INPUT, "Lock");
    if  ((lock != NULL) && (*lock == 2) )
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	int err = 0;
	
	double currentPosition;
	
	float offset_mm = 0.f;
	
	if(performCheck()!=0){
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	

	if(!data ||
			!data->hasKey(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter not present";
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Offset millimeters parameter  not present" );
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	/*
	if(!data->isDoubleValue(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter is not a Double data type";
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	 */
	offset_mm = 0;
	offset_mm = static_cast<float>(data->getDoubleValue(CMD_ACT_MM_OFFSET));
	//SCLAPP_<<"offset_mm:"<<offset_mm;

	if(std::isnan(offset_mm)==true){
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Offset millimeters parameter is not a valid double number (nan?)");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	AbstractActuatorCommand::acquireHandler();

	currentPosition=*o_position;
	double newPosition=currentPosition+offset_mm;
	if(!getDeviceDatabase()->isValid("position", newPosition)){ // nota: *o_position aggiornata inizialmente da AbstractActuatorCommand::acquireHandler();
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Final set point %1% outside the maximum/minimum 'position' = tolerance \"max_position\":%2% \"min_position\":%3%" , % (currentPosition + offset_mm) % max_position % min_position));
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	std::string retStr="NULLA";
	double realSpeed=0;
	if ((err = actuator_drv->getParameter(*axID,"speed",retStr)) != 0)
	{
	   	//SCLDBG_ << "ALEDEBUG failed to read speed from driver";
	   	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Warning cannot know the real speed of motor. Using setTimeout parameter for calculating timeout");
	   	realSpeed=0;
    }
    else
    {
    	SCLDBG_ << "ALEDEBUG driver said speed is " << retStr;
    	realSpeed=atof(retStr.c_str());
    }
	SCLDBG_ << "Compute timeout for moving relative = " << offset_mm;

	uint64_t computed_timeout; // timeout will be expressed in [ms]
	if (realSpeed != 0)
	{
		computed_timeout  = uint64_t((offset_mm / realSpeed)*1000000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS;
		computed_timeout=std::max(computed_timeout,(uint64_t)*p_setTimeout);
		SCLDBG_ << "Calculated timeout is = " << computed_timeout;
	}
	else
	{
		computed_timeout = (uint64_t)*p_setTimeout;
		SCLDBG_ << "Standard  timeout used = " << computed_timeout;
	}

	
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);



	
	if(*o_stby==0){
			// we are in standby only the SP is set
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("we are in standby we cannot start move to '%1%'",%*i_position));
			BC_FAULT_RUNNING_PROPERTY;
			return;
		}
	SCLDBG_ << "Move to position " << newPosition << " reading type " << readTyp;

	*i_position=newPosition; 
	getAttributeCache()->setInputDomainAsChanged();
	SCLDBG_ << "o_position_sp is = " << *i_position;
	if((err = actuator_drv->moveRelative(*axID,offset_mm)) != 0) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("axis %1% cannot perform relative move to '%2%' mm",%*axID %offset_mm));
		setStateVariableSeverity(StateVariableTypeAlarmCU,"user_command_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	*o_home=false;
	sleep(1);
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command move relative :%1% timeout %2%") % offset_mm % computed_timeout) );
	//************* actuator_drv->accessor->base_opcode_priority=100; ********************* commentato
	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdACTMoveRelative::acquireHandler() {          //OK

	//acquire the current readout
	AbstractActuatorCommand::acquireHandler();
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdACTMoveRelative::ccHandler() {
	checkEndMove();

}

bool own::CmdACTMoveRelative::timeoutHandler() {

	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	double delta_position_reached = std::abs(*i_position - *o_position);
	//uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	//move the state machine on fault
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("timeout, delta position remaining %1%",%delta_position_reached));


	if(delta_position_reached <= *p_resolution) {

		SCLDBG_ << "[metric] Setpoint reached on timeout with set point " << *i_position<< " readout position" << *o_position << " resolution" << *p_resolution <<  " in " << elapsed_msec << " milliseconds";
		//the command is endedn because we have reached the affinitut delta set

		BC_END_RUNNING_PROPERTY;


	}else {
		//uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		SCLERR_ << "[metric] Setpoint not reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
		//FIRE OUT OF SET
			//BC_END_RUNNING_PROPERTY; // ************* commentato *******************
		BC_FAULT_RUNNING_PROPERTY;

	}
	return false;
}
