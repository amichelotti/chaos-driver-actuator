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
	CMDCUDBG_ << "loading pointer for output channel";

	o_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");
	o_status = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "status");
    o_alarms = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "alarms");
	
	//get pointer to the output datase variable
	chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = driverAccessorsErogator->getAccessoInstanceByIndex(0);
	if(actuator_accessor != NULL) {
	  if(actuator_drv == NULL){
	    actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
	  }
	}
}

// return the implemented handler
uint8_t AbstractActuatorCommand::implementedHandler() {
	return  chaos_batch::HandlerType::HT_Set |chaos_batch::HandlerType::HT_Correlation;
}

void AbstractActuatorCommand::ccHandler() {
	
}

void AbstractActuatorCommand::getState(int32_t axisID,int& current_state, std::string& current_state_str) {
	CHAOS_ASSERT(actuator_drv)
	int err = 0;
	std::string state_str;
	int32_t i_driver_timeout = getAttributeCache()->getValue<int32_t>(DOMAIN_INPUT, "driver_timeout");
	if((err=actuator_drv->getState(axisID,&current_state, state_str)) != 0) {
		setWorkState(false);
		CMDCUERR_ << boost::str( boost::format("Error getting the actuator state = %1% ") % err);
	}

}

void AbstractActuatorCommand::setWorkState(bool working_flag) {
	int64_t *o_dev_state = getAttributeCache()->getRWPtr<int64_t>(DOMAIN_OUTPUT, "dev_state");
	*o_dev_state = working_flag;
}
