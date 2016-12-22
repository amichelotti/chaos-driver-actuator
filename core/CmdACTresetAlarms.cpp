/*
CmdACTresetAlarms.cpp
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
#include "CmdACTresetAlarms.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTresetAlarms,CMD_ACT_RESETALARMS_ALIAS,
			"Reset the alarms on the actuator",
			"23db61e1-919f-48d8-b8a9-e9dc44a04688")
BATCH_COMMAND_ADD_INT64_PARAM(CMD_ACT_ALRM,"alarm mask",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
//uint8_t own::CmdACTresetAlarms::implementedHandler(){
//	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
// set handler
void own::CmdACTresetAlarms::setHandler(c_data::CDataWrapper *data) {
	int err;
	AbstractActuatorCommand::setHandler(data);
       
 	
        
        setAlarmSeverity("reset_alarms_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);
        
        
        if(!data ||
            !data->hasKey(CMD_ACT_ALRM)) {
            SCLERR_ << "Reset alarms parameter not present";
            //setAlarmSeverity("ResetAlarms_not_present", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing reset alarms: argument non present")) );
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        if(!data->isInt64Value(CMD_ACT_ALRM)) {
            SCLERR_ << "Reset alarms parameter is not an integer 64 data type";
            //setAlarmSeverity("Reset alarms_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing reset alarms: argument is not an integer 64 type")) );
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
	int64_t alarmMask = data->getInt64Value(CMD_ACT_ALRM);
        if(isnan(alarmMask)==true)
        {
            SCLERR_ << "Reset alarms parameter is nan";
            //setAlarmSeverity("Reset_alarms_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing reset alarms: argument is a nan")) );
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
	SCLDBG_ << "ALEDEBUG Reset Alarms set handler "<< alarmMask ;
        
     /*   if(*o_stby){
            // we are in standby only the SP is set
            SCLDBG_ << "we are in standby we cannot start reset alarms operation: ";
            setWorkState(false);
            BC_END_RUNNING_PROPERTY;
            return;
        } 
*/

            SCLERR_ << "resetAlarms before sending to interface " << *axID << actuator_drv;
        if((err = actuator_drv->resetAlarms(*axID,alarmMask)) != 0) {
                //LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% resetting alarms") % err));
            SCLERR_ << "resetAlarms failed";
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing reset alarms: operation failed")) );
            setAlarmSeverity("reset_alarms_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }	
            SCLERR_ << "resetAlarms after sending to interface";
	//actuator_drv->accessor->base_opcode_priority=100;
        setWorkState(true);
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command homing of type: timeout")) );
        BC_EXEC_RUNNING_PROPERTY;
	return;
}

// empty acquire handler
void own::CmdACTresetAlarms::acquireHandler() {
 //force output dataset as changed
	SCLDBG_ << "ALEDEBUG Reset Alarms acquire handler ";
        getAttributeCache()->setOutputDomainAsChanged();
}

// empty correlation handler
void own::CmdACTresetAlarms::ccHandler() {
	SCLDBG_ << "ALEDEBUG Reset Alarms CC handler ";
        setWorkState(false);
        BC_END_RUNNING_PROPERTY;
	return;
}
// empty timeout handler
bool own::CmdACTresetAlarms::timeoutHandler() {
	SCLDBG_ << "ALEDEBUG Reset Alarms timeout handler ";
	return false;
}
