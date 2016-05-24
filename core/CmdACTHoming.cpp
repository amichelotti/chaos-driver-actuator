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
	int state,*tmpInt;
	std::string state_str;
	AbstractActuatorCommand::setHandler(data);
    	int32_t homType = data->getInt32Value(CMD_ACT_HOMINGTYPE);
	homingTypeVar=homType;
	SCLDBG_ << "fetch state readout";
	tmpInt =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
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
	//actuator_drv->setTimeoutHoming(30000);
    if(err = actuator_drv->homing((::common::actuators::AbstractActuator::homingType) homType) < 0) 
    {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% while homing") % err));
    }
	setWorkState(false);
	BC_EXEC_RUNNIG_PROPERTY;
}




//  acquire handler
void own::CmdACTHoming::acquireHandler() {
	int err;
	double position;
	SCLDBG_ << "Start Acquire Handler " ;

	if ((err = actuator_drv->getPosition(readTyp,&position)) !=0) 
	{
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
	} 
	else 
        {
		SCLDBG_ << "Homing acquire after getPosition "<<err ;
                // *o_position = position;
	}

		SCLDBG_ << "Homing acquire before sending homing again" ;
		setWorkState(true);
	if(err = actuator_drv->homing((::common::actuators::AbstractActuator::homingType) homingTypeVar) < 0)
    	{
                LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% while homing") % err));
    	}
    	homResult=err;

    	//force output dataset as changed
    	getAttributeCache()->setOutputDomainAsChanged();


}
//  correlation handler
void own::CmdACTHoming::ccHandler() {
		SCLDBG_ <<" CC Handler homResults " << homResult ;
	if (homResult == 0)
        {
		BC_END_RUNNIG_PROPERTY;
		setWorkState(false);
	}
}
// empty timeout handler
bool own::CmdACTHoming::timeoutHandler() {

}
