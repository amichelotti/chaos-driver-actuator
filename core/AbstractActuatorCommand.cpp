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
	    i_bypass =getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "bypass");

                chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = *i_bypass&&(driverAccessorsErogator->getAccessoInstanceByIndex(1))?driverAccessorsErogator->getAccessoInstanceByIndex(1):driverAccessorsErogator->getAccessoInstanceByIndex(0);

	//get pointer to the output datase variable
	if(actuator_accessor != NULL) {
	  if(actuator_drv == NULL){
	    actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
	  }
	}
}

// return the implemented handler
uint8_t AbstractActuatorCommand::implementedHandler() { 
	return  chaos_batch::HandlerType::HT_Set |chaos_batch::HandlerType::HT_Acquisition|chaos_batch::HandlerType::HT_Correlation; // modificata ****************************************************
}

void AbstractActuatorCommand::acquireHandler(){// ******** Aggiunta questa definizione!!! ********
    
    int err;
    int state=0;
    double position;
    std::string desc;
       
    std::string state_str;
	
    SCLDBG_ << "AbstractActuatorCommand::acquireHandler() " ;
    //acquire the current readout
    SCLDBG_ << "fetch current readout";
        
    if((err = actuator_drv->getAlarms(*axID,o_alarms,desc))==0){
        LOG_AND_TROW(SCLERR_, 2, boost::str(boost::format("Error fetching alarms readout with code %1%") % err));
                        
    }
        
    o_alarm_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "alarmStr");
    strncpy(o_alarm_str, desc.c_str(), 256);
        
    if((err = actuator_drv->getState(*axID,&state, state_str))==0) {
        *o_status_id = state;
        //copy up to 255 and put the termination character
        strncpy(o_status, state_str.c_str(), 256);
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
        
//	if((slow_acquisition_index = !slow_acquisition_index)) {
//	
//	} else {
//	    SCLDBG_ << "fetch alarms readout";
//		if((err = actuator_drv->getAlarms(*axID,o_alarms,desc))){
//			LOG_AND_TROW(SCLERR_, 2, boost::str(boost::format("Error fetching alarms readout with code %1%") % err));
//		}
//                o_alarm_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "alarmStr");
//		strncpy(o_alarm_str, desc.c_str(), 256);
//	}
        
    CMDCUDBG_ << "current ->" << *o_current;
    CMDCUDBG_ << "current_sp ->" << *i_current;
    CMDCUDBG_ << "voltage ->" << *o_voltage;
    CMDCUDBG_ << "polarity ->" << *o_pol;
    CMDCUDBG_ << "alarms ->" << *o_alarms;
    CMDCUDBG_ << "stby -> " << *o_stby;
    
    //force output dataset as changed
  
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
