/*
 *	CmdACTMoveAbsolute.cpp
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
#include "CmdACTMoveAbsolute.h"

#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

#define SCLAPP_ INFO_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "


namespace own =  driver::actuator;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;


BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTMoveAbsolute,CMD_ACT_MOVE_ABSOLUTE_ALIAS,
		"Move from current position to an absolute position (mm)",
		"0cf52f76-55eb-4rt3-8712-3d54484043d8")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_MM_OFFSET, "position mm",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

//// return the implemented handler           //************************** commentato *****************************
//uint8_t own::CmdACTMoveAbsolute::implementedHandler(){
//    return	AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
//uint64_t computed_timeout;

void own::CmdACTMoveAbsolute::setHandler(c_data::CDataWrapper *data) {
	//    chaos::common::data::RangeValueInfo position_sp_attr_info;
	//    chaos::common::data::RangeValueInfo attributeInfo;

	double currentPosition;
	int err = 0;

	float positionToReach = 0.f;
	AbstractActuatorCommand::setHandler(data);
	readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;

	setWorkState(true);
	if(performCheck()!=0){
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	//    axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
	//    o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
	//    o_position_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp");
	//    i_speed = (double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");
	//    i_command_timeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "command_timeout");
	//    __i_delta_setpoint = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__delta_setpoint");
	//    __i_setpoint_affinity = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__setpoint_affinity");
	//    tmpInt = (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
	//    readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;

	setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);



	// ********************* a cosa servono **********************
	//SCLDBG_<<"minimum working value:"<<*p_minimumWorkingValue;
	//SCLDBG_<<"maximum, working value:"<<*p_maximumWorkingValue;


	if(!data ||
			!data->hasKey(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Position millimeters parameter not present";
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Position millimeters parameter is missing");

		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	/*
    if(!data->isDoubleValue(CMD_ACT_MM_OFFSET)) {
	SCLERR_ << "Position millimeters parameter is not a Double data type";
	BC_FAULT_RUNNING_PROPERTY;
	return;
    }
  i*/      
	positionToReach = static_cast<float>(data->getDoubleValue(CMD_ACT_MM_OFFSET));
	if(isnan(positionToReach)==true){
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Position parameter is not a valid double number (nan?)" );

		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	// Controllo setpoint finale: se tale valore appartiene al range [min_position-tolmin,max_position+tolmax]

	if ((positionToReach > max_position) || (positionToReach< min_position))
	{
		SCLERR_ << "Finale position "<<positionToReach<< " out of range ( " << min_position << ","<< max_position <<") the command won't be executed";
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Final set point %1% outside the maximum/minimum 'position_sp' = tolerance \"max_position\":%2% \"min_position\":%3%" , % positionToReach % max_position % min_position));
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	AbstractActuatorCommand::acquireHandler();

	// Ma lo spostamento da effettuare e' maggiore dello spostamento minimo *p_resolution?



	currentPosition=*o_position;
	double deltaPosition = std::abs(positionToReach-currentPosition);

	SCLDBG_ << "compute timeout for moving Absolute = " << positionToReach;
	std::string retStr="NULLA";
	double realSpeed=0;
    if ((err = actuator_drv->getParameter(*axID,"speed",retStr)) != 0)
    {
    	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Warning cannot know the real speed of motor. Using DB value instead");
    	realSpeed=(*i_speed);

    }
    else
    {
    	SCLDBG_ << "ALEDEBUG driver said speed is " << retStr << endl;
    	realSpeed=atof(retStr.c_str());
    }
	//numero di secondi, dopo lo moltiplichiamo per 1 milione (volendo da micro)
	uint64_t computed_timeout; // timeout will be expressed in [ms]
	if (realSpeed != 0)
	{
		computed_timeout  = uint64_t((deltaPosition / realSpeed)*1000000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS;
		computed_timeout = std::max(computed_timeout,(uint64_t)*p_setTimeout);

	}   else computed_timeout=(uint64_t)*p_setTimeout;

	SCLDBG_ << "Calculated timeout is = " << computed_timeout;
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);

	//slow_acquisition_index = false;

	if(*o_stby==0){
		// we are in standby only the SP is set
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("we are in standby we cannot start move to '%1%'",%*i_position));

		BC_END_RUNNING_PROPERTY;
		return;
	}

	SCLDBG_ << "Move to position " << positionToReach << "reading type " << readTyp;

	*i_position=positionToReach;
	getAttributeCache()->setInputDomainAsChanged();
	SCLDBG_ << "o_position_sp is = " << *i_position;
	if((err = actuator_drv->moveAbsoluteMillimeters(*axID,positionToReach)) != 0) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("axis %1% cannot perform absolute move to '%2%'",%*axID %positionToReach));
		setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command move absolute :%1% timeout %2%") % positionToReach % computed_timeout) );
	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdACTMoveAbsolute::acquireHandler() {   //************ modificato in maniera analoga al file CmdACTMoveRelative.cpp **********
	//acquire the current readout
	AbstractActuatorCommand::acquireHandler();
	//force output dataset as changed
	//	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdACTMoveAbsolute::ccHandler() {
	checkEndMove();

}

bool own::CmdACTMoveAbsolute::timeoutHandler() {
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	double delta_position_reached = std::abs(*i_position - *o_position);

	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("timeout, delta position remaining %1%",%delta_position_reached));

	SCLDBG_ << "  TIM MoveABsolute Readout: "<< *o_position <<" SetPoint: "<< *i_position<<" Delta to reach: " << delta_position_reached;
	SCLDBG_ << "  TIM MoveABsolute  resolution: " << *p_resolution;
	if(delta_position_reached <= *p_resolution) {
		SCLDBG_ << "[metric] Setpoint reached on timeout with set point " << *i_position<< " readout position" << *o_position << " resolution" << *p_resolution << " in " << elapsed_msec << " milliseconds";

	}else {

		SCLERR_ << "[metric] Setpoint NOT reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
		setStateVariableSeverity(StateVariableTypeAlarmCU,"position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
	}
	BC_END_RUNNING_PROPERTY;
	return false;
}
