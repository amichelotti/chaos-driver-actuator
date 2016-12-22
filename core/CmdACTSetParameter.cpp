/*
CmdACTSetParameter.cpp
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
#include "CmdACTSetParameter.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTSetParameter) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTSetParameter) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTSetParameter) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTSetParameter,CMD_ACT_SETPARAMETER_ALIAS,
			"Set a inner parameter of the actuator",
			"0f5cf957-e671-4ccc-8f02-c27d87a82f0a")
BATCH_COMMAND_ADD_STRING_PARAM(CMD_ACT_PARNAME,"name of parameter to be set",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_ADD_STRING_PARAM(CMD_ACT_SETPAR_VALUE,"value of the parameter",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
uint8_t own::CmdACTSetParameter::implementedHandler(){
	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
}
// empty set handler
void own::CmdACTSetParameter::setHandler(c_data::CDataWrapper *data) {
	std::string parName,value;
	int err;
	

	AbstractActuatorCommand::setHandler(data);
	SCLDBG_ << "check data";
	if(!data ||
	   !data->hasKey(CMD_ACT_PARNAME)) {
		SCLERR_ << "parameter name  not present";
 		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"parameter name not present.Command Set Parameter aborted" ); // ********** aggiunto **************
		BC_END_RUNNING_PROPERTY;
		return;
	}
	if(!data ||
	   !data->hasKey(CMD_ACT_SETPAR_VALUE)) {
		SCLERR_ << "parameter value  not present";
 		metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"parameter value not present.Command Set Parameter aborted" ); // ********** aggiunto **************
		BC_END_RUNNING_PROPERTY;
		return;
	}
	parName= static_cast<std::string>(data->getStringValue(CMD_ACT_PARNAME));
	value= static_cast<std::string>(data->getStringValue(CMD_ACT_SETPAR_VALUE));
        //SCLDBG_ << "ALEDEBUG axisID to send " << *axID ;
        
        if(*o_stby){
        // we are in standby only the SP is set
            SCLDBG_ << "we are in standby we cannot start set parameter command: ";
            setWorkState(false);
            BC_END_RUNNING_PROPERTY;
            return;
        } 
        
	if((err = actuator_drv->setParameter(*axID,parName,value)) != 0) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% setting parameter %2") % err % parName));
	}
}
// empty acquire handler
void own::CmdACTSetParameter::acquireHandler() {
}
// empty correlation handler
void own::CmdACTSetParameter::ccHandler() {
BC_END_RUNNING_PROPERTY;
}
// empty timeout handler
bool own::CmdACTSetParameter::timeoutHandler() {
}
