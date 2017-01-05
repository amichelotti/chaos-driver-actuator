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
#define CMDCUINFO_ INFO_LOG(AbstractActuatorCommand)
#define CMDCUDBG_ DBG_LOG(AbstractActuatorCommand)
#define CMDCUERR_ ERR_LOG(AbstractActuatorCommand)

using namespace driver::actuator;

namespace chaos_batch = chaos::common::batch_command;

AbstractActuatorCommand::AbstractActuatorCommand() {
  actuator_drv = NULL;
}
AbstractActuatorCommand::~AbstractActuatorCommand() {
  if(actuator_drv)
    delete (actuator_drv);
  actuator_drv = NULL;
}

void AbstractActuatorCommand::setHandler(c_data::CDataWrapper *data) {
	CMDCUDBG_ << "setting ";
        int *tmpInt;
        
        
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

        p_stopCommandInExecution = getAttributeCache()->getRWPtr<bool>(DOMAIN_OUTPUT, "stopHoming");
        
        axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
        tmpInt =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ; 
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt; 
        
	i_speed = ( double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");
        highspeed_homing= ( double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "highspeedhoming");
	//i_command_timeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "command_timeout");
	__i_delta_setpoint = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__delta_setpoint");
	__i_setpoint_affinity = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__setpoint_affinity");
        //p_minimumWorkingValue = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "minimumWorkingValue");
        p_minimumWorkingValue=0;
        p_maximumWorkingValue=0;
        //p_maximumWorkingValue = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "maximumWorkingValue");
        //p_warningThreshold = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "warningThreshold");
        
        p_setTimeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "setTimeout"); 
        
        p_resolution = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "resolution");
        
        //...DA INIZIALIZZARE axID, i_bypass (entrambi dovrebbe essere di tipo DOMAIN_INPUT)
        //...
        //...
	s_bypass =getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "bypass");
        //...
        //...
        //...
        
        //get pointer to the output datase variable
        chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = *s_bypass&&(driverAccessorsErogator->getAccessoInstanceByIndex(1))?driverAccessorsErogator->getAccessoInstanceByIndex(1):driverAccessorsErogator->getAccessoInstanceByIndex(0);
        if(actuator_accessor != NULL) {
	  if(actuator_drv == NULL){
	    actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
	  }
	}
        setWorkState(false);
}

// return the implemented handler
uint8_t AbstractActuatorCommand::implementedHandler() { 
	return  chaos_batch::HandlerType::HT_Set |chaos_batch::HandlerType::HT_Acquisition|chaos_batch::HandlerType::HT_Correlation; // modificata ****************************************************
}

void AbstractActuatorCommand::DecodeAndRaiseAlarms(uint64_t mask)
{
if (mask & ::common::actuators::ACTUATOR_CANBUS_ERROR!= 0)
	setAlarmSeverity("DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);
if (mask & ::common::actuators::ACTUATOR_SHORT_CIRCUIT!= 0)
	setAlarmSeverity("DRIVER_SHORT_CIRCUIT_PROTECTION", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_SHORT_CIRCUIT_PROTECTION", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_INVALID_SETUP_DATA!= 0)
	setAlarmSeverity("DRIVER_INVALID_SETUP_DATA", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_INVALID_SETUP_DATA", chaos::common::alarm::MultiSeverityAlarmLevelClear);


if (mask & ::common::actuators::ACTUATOR_CONTROL_ERROR!= 0)
	setAlarmSeverity("DRIVER_CONTROL_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_CONTROL_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);


if (mask & ::common::actuators::ACTUATOR_SERIAL_COMM_ERROR!= 0)
	setAlarmSeverity("DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_COMMUNICATION_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_HALL_SENSOR_MISSING!= 0)
	setAlarmSeverity("DRIVER_HALL_SENSOR_MISSING", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_HALL_SENSOR_MISSING", chaos::common::alarm::MultiSeverityAlarmLevelClear);


if (mask & ::common::actuators::ACTUATOR_OVER_CURRENT!= 0)
	setAlarmSeverity("DRIVER_OVER_CURRENT", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_OVER_CURRENT", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_I2T!= 0)
	setAlarmSeverity("DRIVER_I2T_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVER_I2T_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_OVERTEMP_MOTOR!= 0)
	setAlarmSeverity("MOTOR_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("MOTOR_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_OVERTEMP_DRIVE!= 0)
	setAlarmSeverity("DRIVE_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("DRIVE_OVER_TEMPERATURE", chaos::common::alarm::MultiSeverityAlarmLevelClear);


if (mask & ::common::actuators::ACTUATOR_OVERVOLTAGE!= 0)
	setAlarmSeverity("OVER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("OVER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_UNDERVOLTAGE!= 0)
	setAlarmSeverity("UNDER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("UNDER_VOLTAGE_ALARM", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_COMMANDERROR!= 0)
	setAlarmSeverity("DRIVER_COMMAND_ERROR", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("", chaos::common::alarm::MultiSeverityAlarmLevelClear);

if (mask & ::common::actuators::ACTUATOR_ALARMS_READING_ERROR!= 0)
	setAlarmSeverity("READING_ALARM_PROBLEM", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
else
	setAlarmSeverity("READING_ALARM_PROBLEM", chaos::common::alarm::MultiSeverityAlarmLevelClear);


}
void AbstractActuatorCommand::acquireHandler(){// ******** Aggiunta questa definizione!!! ********
    
    int err;
    
    double position;
   
    std::string descStr;
    uint64_t tmp_uint64;
    int state=0;
    
	
    CMDCUDBG_ << "AbstractActuatorCommand::acquireHandler() " ;
    //acquire the current readout
    CMDCUDBG_ << "fetch current readout";
        
    if((err = actuator_drv->getAlarms(*axID,&tmp_uint64,descStr))==0){
        *o_alarms = tmp_uint64;
        //copy up to 255 and put the termination character
        strncpy(o_alarm_str, descStr.c_str(), 256);
	//decode and raise alarms
        DecodeAndRaiseAlarms(tmp_uint64);
    }
    else{
        CMDCUERR_<<boost::str( boost::format("Error calling driver on get alarms readout with code %1%") % err);
    }
   
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


        strncpy(o_status_str, descStr.c_str(), 256);
    } else {
        CMDCUERR_ <<boost::str( boost::format("Error calling driver on get state readout with code %1%") % err);
    }
        
    if ((err = actuator_drv->getPosition(*axID,readTyp,&position))==0) {
        //LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
        *o_position = position;
    } else {
        //*o_position = position;
        CMDCUERR_ <<boost::str( boost::format("Error calling driver on get Position readout with code %1%") % err);
    }
    
    //force output dataset as changed
}

void AbstractActuatorCommand::ccHandler() {
    getAttributeCache()->setOutputDomainAsChanged();
}

void AbstractActuatorCommand::getState(int32_t axisID,int& current_state, std::string& current_state_str) {
        CHAOS_ASSERT(actuator_drv)
        int err = 0;
	std::string state_str;
	//int32_t i_driver_timeout = getAttributeCache()->getValue<int32_t>(DOMAIN_INPUT, "driver_timeout"); // *************** commentato *************
	if((err=actuator_drv->getState(axisID,&current_state, state_str)) != 0) {
		//setWorkState(false);    // *************** commentato *****************
            CMDCUERR_ << boost::str( boost::format("Error getting the actuator state = %1% ") % err);
	}
}

void AbstractActuatorCommand::setWorkState(bool working_flag) {
//	int64_t *o_dev_state = getAttributeCache()->getRWPtr<int64_t>(DOMAIN_OUTPUT, "dev_state"); // *************** commentato *****************
//	*o_dev_state = working_flag; // *************** commentato *****************
    setBusyFlag(working_flag);
}
