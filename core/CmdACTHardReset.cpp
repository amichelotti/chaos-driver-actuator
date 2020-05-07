/*
CmdACTHardReset.cpp
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
#include "CmdACTHardReset.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTHardReset) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTHardReset) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTHardReset) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTHardReset,CMD_ACT_HARDRESET_ALIAS,
			"Hard Reset on the driver of the actuator",
			"25e1c0fe-775e-40e4-9b96-ae7e797c1487")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_HARDRESET_MODE,"mode of the hard reset",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)

BATCH_COMMAND_CLOSE_DESCRIPTION()


// set handler
void own::CmdACTHardReset::setHandler(c_data::CDataWrapper *data) {
	int err;
	AbstractActuatorCommand::setHandler(data);
    SCLDBG_ << "HardReset set handler " ;
        if(!data ||
                !data->hasKey(CMD_ACT_HARDRESET_MODE)) {
                metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"mode parameter of hard reset  not specified ");
                BC_FAULT_RUNNING_PROPERTY;
                return;
        }

        if(!data->isInt32Value(CMD_ACT_HARDRESET_MODE)) {
                metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"mode parameter is not an integer data type");
                BC_FAULT_RUNNING_PROPERTY;
                return;
        }


	int32_t resetMode = data->getInt32Value(CMD_ACT_HARDRESET_MODE);


    SCLDBG_ << "HardReset set handler " ;
        

        SCLDBG_ << "HardReset before sending to interface " ;
        if((err = actuator_drv->hardreset(*axID,resetMode)) != 0) {
                //LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% resetting alarms") % err));
            SCLERR_ << "HardReset failed";
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing reset alarms: operation failed")) );
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }	
            SCLERR_ << "HardReset after sending to interface";
	*o_lasthoming=0;
	getAttributeCache()->setOutputDomainAsChanged();
	//actuator_drv->accessor->base_opcode_priority=100;
        BC_NORMAL_RUNNING_PROPERTY;
	return;
}

// empty acquire handler
void own::CmdACTHardReset::acquireHandler() {
 //force output dataset as changed
	SCLDBG_ << "Hard Reset  acquire handler ";
        
}

// empty correlation handler
void own::CmdACTHardReset::ccHandler() {
	SCLDBG_ << "Hard Reset CC handler ";
        BC_END_RUNNING_PROPERTY;
	return;
}
// empty timeout handler
bool own::CmdACTHardReset::timeoutHandler() {
	SCLDBG_ << "Hard Reset timeout handler ";
	return false;
}
