/*
 *	AbstractActuatorCommand.cpp
 *	!CHOAS
 *	Created by Claudio Bisegni.
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

#include "AbstractActuatorCommand.h"
#include <boost/format.hpp>
#define CMDCUINFO_ INFO_LOG(AbstractActuatorCommand) <<"["<<getDeviceID()<<"]"
#define CMDCUDBG_ DBG_LOG(AbstractActuatorCommand) <<"["<<getDeviceID()<<"]"
#define CMDCUERR_ ERR_LOG(AbstractActuatorCommand) <<"["<<getDeviceID()<<"]"

using namespace driver::actuator;

namespace chaos_batch = chaos::common::batch_command;
using namespace chaos::cu::control_manager;


AbstractActuatorCommand::AbstractActuatorCommand() {
	actuator_drv = NULL;
		
}
AbstractActuatorCommand::~AbstractActuatorCommand() {


	if(actuator_drv)
		delete (actuator_drv);
	actuator_drv = NULL;
}


void AbstractActuatorCommand::checkEndMove(){
	int err;
	//check if we are in the delta of the setpoint to end the command
		double delta_position_reached = std::abs(*i_position - *o_position);
		//SCLDBG_ << "ccH MoveABsolute Readout: "<< *o_position <<" SetPoint: "<< *o_position_sp <<" Delta to reach: " << delta_position_reached << " computed Timeout " << computed_timeout ;
		CMDCUDBG_ << "Readout: "<< *o_position <<" SetPoint: "<< *i_position <<" Delta to reach: " << delta_position_reached;

	if((delta_position_reached=getDeviceDatabase()->compareTo("position",*i_position,*o_position))==0){
		uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		std::stringstream ss;
		ss<< "Setpoint reached set point " << *i_position<< " readout position" << *o_position << " in " << elapsed_msec << " milliseconds";

		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,ss.str() );

		BC_END_RUNNING_PROPERTY;
		return;
	} else {
		CMDCUDBG_ << " checkEndMove getDeviceDatabase()->compareTo returned " << delta_position_reached;
		//setStateVariableSeverity(StateVariableTypeAlarmCU,"position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);

	}
	/*	if(delta_position_reached <= *p_resolution){
				uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
				//the command is endedn because we have reached the affinitut delta set
				CMDCUDBG_ << "[metric ]Set point reached with - delta: "<< delta_position_reached <<" sp: "<< *i_position <<" affinity check " << *p_resolution << " mm in " << elapsed_msec << " milliseconds";
				BC_END_RUNNING_PROPERTY;
				return;
		}
*/
//if (! *o_useUI)
{
		if ((((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION)==0) ||(((*o_status_id) & ::common::actuators::ACTUATOR_POWER_SUPPLIED)==0)){
			CMDCUDBG_ << " checkEndMove : motor moving state is " << (!(((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION) == 0));
			CMDCUDBG_ << " checkEndMove : motor power supply state is " << (!(((*o_status_id) & ::common::actuators::ACTUATOR_POWER_SUPPLIED) == 0));
			setStateVariableSeverity(StateVariableTypeAlarmCU,"position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("checkendmove stopping motion because motors seems not moving anymore,or without power motor axis %1% has stopped before reach setpoint, delta %2% %3%",%*axID %delta_position_reached %(((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION)==0)));
			
			if ((err=actuator_drv->stopMotion(*axID))!= 0){
						metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("cannot stop motion on axis '%1%'",%*axID));
						//supported in framework whenever FAULT_RUNNING 
		//				setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
						BC_FAULT_RUNNING_PROPERTY;
						return;
			}
			BC_FAULT_RUNNING_PROPERTY;
		}
}

}

int AbstractActuatorCommand::performCheck(){
	getDeviceDatabase()->getAttributeRangeValueInfo("position", position_info);

	// REQUIRE MIN MAX SET IN THE MDS

	if (position_info.maxRange.size()) {
		max_position = atof(position_info.maxRange.c_str());
		CMDCUDBG_ << "max_position max=" << max_position;

	} else {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined maximum 'position' attribute, quitting command" );

		return -1;
	}

	// REQUIRE MIN MAX POSITION IN THE MDS
	if (position_info.minRange.size()) {
		min_position = atof(position_info.minRange.c_str());

		CMDCUDBG_<< "min_position min=" << min_position;
	} else {
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined minimum 'position' attribute, quitting command" );

		return -2;

	}

	return 0;
}
void AbstractActuatorCommand::setHandler(c_data::CDataWrapper *data) {

	chaos::common::data::CDataWrapper p;
	poi.reset();
	hasPOI=false;
	if(getDeviceLoadParams(p)==0){
		p.getCSDataValue(CMD_ACT_MOVE_POI,poi);
		hasPOI=(poi.getAllKey().size()>0);
		
    }

	o_nswitch=getAttributeCache()->getRWPtr<bool>(DOMAIN_OUTPUT, "NegativeLimitSwitchActive");
	o_pswitch=getAttributeCache()->getRWPtr<bool>(DOMAIN_OUTPUT, "PositiveLimitSwitchActive");
	o_stby=getAttributeCache()->getRWPtr<bool>(DOMAIN_OUTPUT, "powerOn");
	o_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");
	o_status_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "status");
	o_alarms = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "alarms");
	o_alarm_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "alarmStr");
	o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
	i_position = (double*) getAttributeCache()->getRWPtr<double>(DOMAIN_INPUT, "position");
	//o_position_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp");
	// ********* nota: i_position rimpiazza o_position_sp
	//p_stopCommandInExecution = getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "stopCommandInExecution");

	p_stopCommandInExecution = getAttributeCache()->getRWPtr<bool>(DOMAIN_CUSTOM, "stopHoming");
	o_lasthoming = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "LastHomingTime");
	o_useUI  = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "useSteps"); 
	o_home = getAttributeCache()->getRWPtr<bool>(DOMAIN_OUTPUT, "home");

	axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
	tmpInt =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
	readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;


	//i_speed = ( double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");
	

	p_setTimeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "setTimeout");

	p_resolution = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "resolution");
    *o_alarm_str=0;
    *o_status_str=0;
	loggedPositionError = loggedStateError= loggedAlarmError = false;

	//get pointer to the output datase variable
	chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = driverAccessorsErogator->getAccessoInstanceByIndex(0);
	if(actuator_accessor != NULL) {
		if(actuator_drv == NULL){
			actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor,getDeviceID());
		}
	}
}

// return the implemented handler
uint8_t AbstractActuatorCommand::implementedHandler() { 
	return  chaos_batch::HandlerType::HT_Set |chaos_batch::HandlerType::HT_Acquisition|chaos_batch::HandlerType::HT_Correlation; // modificata ****************************************************
}

void AbstractActuatorCommand::DecodeAndRaiseAlarms(uint64_t mask)
{
	if ((mask & ::common::actuators::ACTUATOR_CANBUS_ERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_ALARMS_EMERGENCY_ERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"EMERGENCY_LOCK_ENABLED", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"EMERGENCY_LOCK_ENABLED", chaos::common::alarm::MultiSeverityAlarmLevelClear);
	if ((mask & ::common::actuators::ACTUATOR_SHORT_CIRCUIT)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_SHORT_CIRCUIT_PROTECTION", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_SHORT_CIRCUIT_PROTECTION", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_INVALID_SETUP_DATA)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_INVALID_SETUP_DATA", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_INVALID_SETUP_DATA", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	if ((mask & ::common::actuators::ACTUATOR_CONTROL_ERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_CONTROL_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_CONTROL_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	if ((mask & ::common::actuators::ACTUATOR_SERIAL_COMM_ERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_HALL_SENSOR_MISSING)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_HALL_SENSOR_MISSING", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_HALL_SENSOR_MISSING", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	if ((mask & ::common::actuators::ACTUATOR_OVER_CURRENT)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_OVER_CURRENT", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_OVER_CURRENT", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_I2T)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_I2T_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_I2T_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_OVERTEMP_MOTOR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"MOTOR_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"MOTOR_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_OVERTEMP_DRIVE)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVE_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVE_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelClear);


	if ((mask & ::common::actuators::ACTUATOR_OVERVOLTAGE)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"OVER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"OVER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_UNDERVOLTAGE)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"UNDER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"UNDER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_COMMANDERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"DRIVER_COMMAND_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"", chaos::common::alarm::MultiSeverityAlarmLevelClear);

	if ((mask & ::common::actuators::ACTUATOR_ALARMS_READING_ERROR)!= 0)
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"READING_ALARM_PROBLEM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
	else
		setStateVariableSeverity(StateVariableTypeAlarmDEV,"READING_ALARM_PROBLEM", chaos::common::alarm::MultiSeverityAlarmLevelClear);


}
std::string AbstractActuatorCommand::position2POI(float pos){
	ChaosStringVector st=poi.getAllKey();
	for(ChaosStringVector::iterator i=st.begin();i!=st.end();i++){
		float pval=poi.getDoubleValue(*i);
		if(getDeviceDatabase()->compareTo("position",pos,pval)==0){
			return *i;
		}
		
	}
	return std::string();
}

void AbstractActuatorCommand::acquireHandler(){

	int err;

	double position;

	std::string descStr;
	uint64_t tmp_uint64;
	int state=0;



	//CMDCUINFO_ << "ALESTARV before getAlarms ax:" << *axID;
	if((err = actuator_drv->getAlarms(*axID,&tmp_uint64,descStr))==0){
		*o_alarms = tmp_uint64;
		//copy up to 255 and put the termination character
		strncpy(o_alarm_str, descStr.c_str(), 256);
		//decode and raise alarms
		DecodeAndRaiseAlarms(tmp_uint64);
		loggedAlarmError = false;
	}else if(err!=DRV_BYPASS_DEFAULT_CODE){
		CMDCUERR_<<boost::str( boost::format("Error calling driver on get alarms readout with code %1%") % err);
		if (!loggedAlarmError)
		{
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError, CHAOS_FORMAT("axis %1% error getting alarms, err:%2%'", %* axID % err));
			loggedAlarmError = true;
		}
	//	setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY
		return;

	}
	//CMDCUINFO_ << "ALESTARV after getAlarms before getState ax:" << *axID;
	if((err = actuator_drv->getState(*axID,&state, descStr))==0) {
		*o_status_id = state;

		if (state &  ::common::actuators::ACTUATOR_LSP_LIMIT_ACTIVE)
			*o_pswitch=true;
		else
			*o_pswitch=false;

		if (state &  ::common::actuators::ACTUATOR_LSN_LIMIT_ACTIVE)
			*o_nswitch=true;
		else
			*o_nswitch=false;

		if (state &  ::common::actuators::ACTUATOR_POWER_SUPPLIED)
			*o_stby=true;
		else
			*o_stby=false;

		if (state & ::common::actuators::ACTUATOR_UNKNOWN_STATUS)
		{
			*o_lasthoming = 0;
			*o_home = false;
			setStateVariableSeverity(StateVariableTypeAlarmCU, "home_lost", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
		}
		else
		{
			setStateVariableSeverity(StateVariableTypeAlarmCU, "home_lost", chaos::common::alarm::MultiSeverityAlarmLevelClear);
		}

		strncpy(o_status_str, descStr.c_str(), 256);
	} else if(err!=DRV_BYPASS_DEFAULT_CODE) {
		CMDCUERR_ <<boost::str( boost::format("Error calling driver on get state readout with code %1%") % err);
		//setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
		BC_FAULT_RUNNING_PROPERTY
		return;
	}

	readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
	//CMDCUINFO_ << "ALESTARV after getState before getPosition ax:" << *axID;
	if ((err = actuator_drv->getPosition(*axID,readTyp,&position))==0) {
		//LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
		*o_position = position;
		loggedPositionError = false;
		if(hasPOI){
				std::string rets=position2POI(position);
				//CMDCULOG_ <<"POI:'"<<ret<<"' ="<<position<<" polist:"<<poi.getJSONString();

				//getAttributeCache()->setOutputAttributeValue("POI",(void*)ret.c_str(),ret.size()+1);
				getAttributeCache()->setOutputAttributeValue("POI",rets);


		}
	} else if(err!=DRV_BYPASS_DEFAULT_CODE) {
		//*o_position = position;
		CMDCUERR_ <<boost::str( boost::format("Error calling driver on get Position readout with code %1%") % err);
		if (!loggedPositionError)
		{
			metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError, CHAOS_FORMAT("axis %1% error getting position, using type %2%, err:%3%'", %* axID % readTyp % err));
			loggedPositionError = true;
		}
	//	setStateVariableSeverity(StateVariableTypeAlarmCU,"command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);

		return;;

	}
	//CMDCUINFO_ << "ALESTARV after getPosition end of Abstract acquirehandler:" << *axID;




}

void AbstractActuatorCommand::ccHandler() {
	getAttributeCache()->setOutputDomainAsChanged();
}

void AbstractActuatorCommand::getState(int32_t axisID,int& current_state, std::string& current_state_str) {
	CHAOS_ASSERT(actuator_drv)
        		int err = 0;
	std::string state_str;
	//int32_t i_driver_timeout = getAttributeCache()->getValue<int32_t>(DOMAIN_INPUT, "driver_timeout"); // *************** commentato *************
	if(((err=actuator_drv->getState(axisID,&current_state, state_str)) != 0) &&(err!=DRV_BYPASS_DEFAULT_CODE) ) {
		CMDCUERR_ << boost::str( boost::format("Error getting the actuator state = %1% ") % err);
		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("axis %1% error getting state, err:%2%'",%*axID %err));

	}
}
