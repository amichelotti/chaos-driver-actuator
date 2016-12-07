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
	o_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");
	o_status_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "status");
        o_alarms_id = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "alarms");
        o_alarm_str = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "alarmStr");  
        o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
        
        
        //...DA INIZIALIZZARE axID, i_bypass (entrambi dovrebbe essere di tipo DOMAIN_INPUT)
        //...
        //...
	i_bypass =getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "bypass");
        //...
        //...
        //...
        
        //get pointer to the output datase variable
        chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = *i_bypass&&(driverAccessorsErogator->getAccessoInstanceByIndex(1))?driverAccessorsErogator->getAccessoInstanceByIndex(1):driverAccessorsErogator->getAccessoInstanceByIndex(0);
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

void AbstractActuatorCommand::acquireHandler(){// ******** Aggiunta questa definizione!!! ********
    
    int err;
    
    double position;
   
    std::string descStr;
    uint64_t tmp_uint64;
    int state=0;
    
	
    SCLDBG_ << "AbstractActuatorCommand::acquireHandler() " ;
    //acquire the current readout
    SCLDBG_ << "fetch current readout";
        
    if((err = actuator_drv->getAlarms(*axID,&tmp_uint64,descStr))==0){
        *o_alarms_id = tmp_uint64;
        //copy up to 255 and put the termination character
        strncpy(o_alarm_str, descStr.c_str(), 256);
    }
    else{
        CMDCUERR_<<boost::str( boost::format("Error calling driver on get alarms readout with code %1%") % err);
    }
   
    if((err = actuator_drv->getState(*axID,&state, descStr))==0) {
        *o_status_id = state;
        //copy up to 255 and put the termination character
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
          
    CMDCUDBG_ << "position ->" << *o_position;
    CMDCUDBG_ << "state id ->" << *o_status_id;
    CMDCUDBG_ << "state ->" << o_status_str;
    CMDCUDBG_ << "alarms id->" << *o_alarms_id;
    CMDCUDBG_ << "alarms -> " << *o_alarm_str;
    
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
//	int64_t *o_dev_state = getAttributeCache()->getRWPtr<int64_t>(DOMAIN_OUTPUT, "dev_state");
//	*o_dev_state = working_flag;
    setBusyFlag(working_flag);
        
}
