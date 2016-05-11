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
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTHoming,CMD_ACT_HOMING_ALIAS,
			"Calibrate the actuator reaching the home position",
			"f096d258-1690-11e6-8845-1f6ad6d4e676")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_HOMINGTYPE,"homing Type",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
uint8_t own::CmdACTHoming::implementedHandler(){
	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
}
// empty set handler
void own::CmdACTHoming::setHandler(c_data::CDataWrapper *data) 
{
	int err = 0;
	int state;
	std::string state_str;
AbstractActuatorCommand::setHandler(data);
    	int32_t homType = data->getInt32Value(CMD_ACT_HOMINGTYPE);
	SCLDBG_ << "fetch state readout";
	/*
	if((err = actuator_drv->getState(&state, state_str))) 
	{
		SCLDBG_ << "fetch state bad exiting";
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching state readout with code %1%") % err));
	} 
	else 
	{
		SCLDBG_ << "assigning";
		*o_status_id = state;
		SCLDBG_ << "copying string";
		strncpy(o_status, state_str.c_str(), 256);
	}
	if(((*o_status_id)&::common::actuators::ACTUATOR_READY)==0)
	{
        SCLERR_ << boost::str( boost::format("Bad state for moving actuator %1%[%2%]") % o_status % *o_status_id);
	    BC_END_RUNNIG_PROPERTY;
	    return;
    }
*/
	 //check mode parameter
    if(!data->hasKey(CMD_ACT_HOMINGTYPE)) 
    {
            SCLERR_ << "Homing type not present";
            BC_END_RUNNIG_PROPERTY;
            return;
    }
	SCLDBG_ << "Accessing accessor at " << actuator_drv;
	actuator_drv->accessor->base_opcode_priority=100;
	setWorkState(true);
	SCLDBG_ << " Must setup timeout Launching homing driver command";
	//actuator_drv->setTimeoutHoming(30000);
    if(err = actuator_drv->homing((::common::actuators::AbstractActuator::homingType) homType) != 0) 
    {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% while homing") % err));
    }
	setWorkState(false);
	BC_EXEC_RUNNIG_PROPERTY;
}




// empty acquire handler
void own::CmdACTHoming::acquireHandler() {
}
// empty correlation handler
void own::CmdACTHoming::ccHandler() {
}
// empty timeout handler
bool own::CmdACTHoming::timeoutHandler() {

}
