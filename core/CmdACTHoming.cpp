/*
CmdACTHoming.cpp
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
#include "CmdACTHoming.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;

BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTHoming,CMD_ACT_HOMING_ALIAS,
		"Calibrate the actuator reaching the home position",
		"f096d258-1690-11e6-8845-1f6ad6d4e676")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_HOMINGTYPE,"homing Type",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


void own::CmdACTHoming::setHandler(c_data::CDataWrapper *data) 
{
	AbstractActuatorCommand::setHandler(data);

	int err = 0;
	//double max_homing_type=0,min_homing_type=0;
	double currentPosition;
	uint64_t computed_timeout;
	*p_stopCommandInExecution=false;
    
	setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	int32_t *lock=getAttributeCache()->getRWPtr<int32_t>(DOMAIN_INPUT, "Lock");
    if  ((lock != NULL) && (*lock > 0) )
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"action_prevented_by_lock_in_configuration", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		BC_NORMAL_RUNNING_PROPERTY;
		return;
	}


	if(!data ||
			!data->hasKey(CMD_ACT_HOMINGTYPE)) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"homing type not specified ");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	if(!data->isInt32Value(CMD_ACT_HOMINGTYPE)) {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Homing  parameter is not an integer data type");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	int32_t homType = data->getInt32Value(CMD_ACT_HOMINGTYPE);


	SCLDBG_ << "Compute timeout for homing operation of type = " << homType;
	//.......................
	AbstractActuatorCommand::acquireHandler(); // Per aggiornare al momento piu opportuno *o_position, *readTyp
	currentPosition=* o_position;
	std::string retStr="NULLA";
	double realSpeed=0;
	double lengthSlit=100;
	if ((err = actuator_drv->getParameter(*axID,"highspeed_homing",retStr)) != 0)
	{
	    	//SCLDBG_ << "ALEDEBUG failed to read speed from driver";
		if ((err = actuator_drv->getParameter(*axID, "speed", retStr)) != 0)
		{
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning, "Warning cannot know the real highspeedhoming of motor.");
			realSpeed = 0;
		}
		else
		{
			realSpeed = atof(retStr.c_str());
		}

	}
	else
	{
		realSpeed=atof(retStr.c_str());
	}
	if ((err = actuator_drv->getParameter(*axID,"range_slit[mm]",retStr)) != 0)
	{
			//SCLDBG_ << "ALEDEBUG failed to read speed from driver";
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"Warning cannot know the real range of the slit. Using default value of 100");
	}
	else
	{
		SCLDBG_ << "ALEDEBUG driver said range of slit is " << retStr << " mm ";
		lengthSlit=atof(retStr.c_str());
	}



	if (realSpeed!= 0)
	{
	computed_timeout  = uint64_t((lengthSlit / realSpeed)*1000000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS;
		computed_timeout = std::max(computed_timeout,(uint64_t)*p_setTimeout);

	}   else computed_timeout=(uint64_t)*p_setTimeout;

	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);

	SCLDBG_ << "Start homing operation of type " << homType;
	if(*o_stby==0){
		// we are in standby only the SP is set
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"cannot perform homing because in PowerOff");
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	*o_lasthoming = 0;
	if((err = actuator_drv->homing(*axID,(::common::actuators::AbstractActuator::homingType) homType)) < 0)
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("axis %1% cannot perform homing type %2% driver err: %3%") %*axID %homType %err));
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}

	homingTypeVar = homType;
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("axis %1% performing command homing of type:%2% timeout %3%") %*axID % homType % computed_timeout) );
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)500000); //*********** (uint64_t)100000 deve diventare un parametro ************
	BC_NORMAL_RUNNING_PROPERTY;
}

//  acquire handler
void own::CmdACTHoming::acquireHandler() {
	int err;
	//int state;
	//std::string state_str;
	//double position;
	SCLDBG_ << "Start Homing Acquire Handler " ;

	//acquire the current readout
	AbstractActuatorCommand::acquireHandler(); // ********* Necessario per aggiornare solo stato e posizione in questo specifico caso ***************
	//force output dataset as changed

	SCLDBG_ << "Homing acquire before sending homing again";
	if((err = actuator_drv->homing(*axID,(::common::actuators::AbstractActuator::homingType) homingTypeVar)) < 0)
	{
		setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,boost::str( boost::format("axis %1% performing driver command homing of type:%2% err: %3%") %*axID % homingTypeVar % err) );

		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	homResult=err;
	SCLDBG_ <<" HOMERESULT " << homResult ;
	getAttributeCache()->setOutputDomainAsChanged();

}

//  correlation handler
void own::CmdACTHoming::ccHandler() {

	SCLDBG_ <<" CC Handler homResults " << homResult ;
	if (homResult == 0){
		uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		SCLDBG_ << "Homing operation completed in "<< elapsed_msec <<" milliseconds";
		*o_lasthoming = chaos::common::utility::TimingUtil::getTimeStamp();
		*o_home=true;
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,"Homing completed");
                *i_position=0;
 		getAttributeCache()->setInputDomainAsChanged();
		getAttributeCache()->setOutputDomainAsChanged();

		BC_END_RUNNING_PROPERTY;
		return;
	}

	if (*p_stopCommandInExecution) // questa funzione dovrebbe essere considerata solo se e' in esecuzione il comando di stop
		// Il comando di stop potrebbe settare un membro della classe astratta.
	{
		*p_stopCommandInExecution=false;
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,("Homing not completed, because stopped" ));
		setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		SCLDBG_ << "Exit from homing because of actutator is not in motion";
		BC_FAULT_RUNNING_PROPERTY;
	}
	if (((*o_status_id) & ::common::actuators::ACTUATOR_POWER_SUPPLIED)==0){
		int err;
		if ((err=actuator_drv->stopMotion(*axID)!= 0))
		{
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Stopping motion");
			SCLDBG_ <<  "ALEDEBUG stopping because not on power on";
			setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
			BC_FAULT_RUNNING_PROPERTY;
			return;

		}
		BC_END_RUNNING_PROPERTY;

	}


}

// empty timeout handler
bool own::CmdACTHoming::timeoutHandler() {

	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("timeout, position remaining %1%, elapsed time %2%" , % * o_position % elapsed_msec));
	// E' necessario, per come è implmentata la procedura di homing,
	// inviare un comando di stop di movimentazione.
	int err;
	if((err = actuator_drv->stopMotion(*axID)) != 0) {


	}
	SCLDBG_ <<  "ALEDEBUG stopped because of timeout";
	setStateVariableSeverity(StateVariableTypeAlarmCU,"homing_operation_failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"Stopping motion, because timeout during homing");
	*o_home=false;

	BC_FAULT_RUNNING_PROPERTY;
	return false;
}
