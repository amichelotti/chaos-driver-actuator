/*
CmdACTPoweron.cpp
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
#include "CmdACTPoweron.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTPoweron) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTPoweron,CMD_ACT_POWERON_ALIAS,
			"Turn on the power of the actuator",
			"29d39d4f-0311-4774-b0b3-8caa7862193c")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_POWERON_VALUE,"on state",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)

BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
//uint8_t own::CmdACTPoweron::implementedHandler(){
//	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
// empty set handler
void own::CmdACTPoweron::setHandler(c_data::CDataWrapper *data) {
	int err;
	int32_t onState = data->getInt32Value(CMD_ACT_POWERON_VALUE);
	AbstractActuatorCommand::setHandler(data);

	
SCLDBG_   << "Launching poweron in set handler power on in axid "<< *axID << " value " << onState;
	if((err = actuator_drv->poweron(*axID,onState)) != 0) {
            SCLDBG_ << "Error while power on the actuator ";
	
	}
	setWorkState(true);
	BC_NORMAL_RUNNING_PROPERTY;
}
// empty acquire handler
void own::CmdACTPoweron::acquireHandler() {
SCLDBG_   << "ALEDEBUG acquire handler power on";
}
// empty correlation handler
void own::CmdACTPoweron::ccHandler() {
	int err,state;
	std::string state_str;
        SCLDBG_ << "cc handler of power on command" ;
	if((err = actuator_drv->getState(*axID,&state, state_str))) 
	{
		SCLDBG_ << "Error fetching state readout";
	} 
	else 
 	{
		*o_status_id = state;
		//copy up to 255 and put the termination character
		strncpy(o_status_str, state_str.c_str(), 256);
		if(((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)!=0)
		{
			BC_END_RUNNING_PROPERTY;
			setWorkState(false);
		}
			BC_END_RUNNING_PROPERTY;
			setWorkState(false);
	}
}
// empty timeout handler
bool own::CmdACTPoweron::timeoutHandler() {
	int err,state;
	std::string state_str;
	setWorkState(false);
 	if((err = actuator_drv->getState(*axID,&state, state_str)))
        {
                LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching state readout with code %1%") % err));
        }
        else
        {
                *o_status_id = state;
                //copy up to 255 and put the termination character
                strncpy(o_status_str, state_str.c_str(), 256);
                if(((*o_status_id)&::common::actuators::ACTUATOR_POWER_SUPPLIED)!=0)
                {
                        BC_END_RUNNING_PROPERTY;
                }
		else
		{
 			SCLERR_ << "[metric] Power on not achieved before timeout " ;
			BC_FAULT_RUNNING_PROPERTY;
		}
        }

}
