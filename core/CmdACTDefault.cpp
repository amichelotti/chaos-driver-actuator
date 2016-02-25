/*
 *	CmdACTDefault.cpp
 *	!CHOAS
 *	Created by Claudio Bisegni.
 *      Adapted by Alessandro D'Uffizi
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

#include <string.h>
#include "CmdACTDefault.h"
#include <driver/actuator/core/ChaosActuatorInterface.h>

#define LOG_HEAD_CmdACTDefault LOG_TAIL(CmdACTDefault)

#define CMDCU_ INFO_LOG(CmdACTDefault) << "[" << getDeviceID() << "] "
#define CMDCUDBG_ DBG_LOG(CmdACTDefault) << "[" << getDeviceID() << "] "
#define CMDCUERR_ ERR_LOG(CmdACTDefault) << "[" << getDeviceID() << "] "

using namespace driver::actuator;
using namespace chaos::common::data;
using namespace chaos::common::batch_command;
using namespace chaos::cu::control_manager::slow_command;

BATCH_COMMAND_OPEN_DESCRIPTION(driver::actuator::,CmdACTDefault,
                                                          "Default method",
                                                          "e39593b3-db3f-435c-957e-f3496200e2f1")
BATCH_COMMAND_CLOSE_DESCRIPTION()
CmdACTDefault::CmdACTDefault() {
	actuator_drv = NULL;
}

CmdACTDefault::~CmdACTDefault() {
	
}

    // return the implemented handler
uint8_t CmdACTDefault::implementedHandler() {
        //add to default hadnler the acquisition one
	return  AbstractActuatorCommand::implementedHandler() |
    HandlerType::HT_Acquisition;
}

    // Start the command execution
void CmdACTDefault::setHandler(c_data::CDataWrapper *data) {
	
        int *tmpInt;
	AbstractActuatorCommand::setHandler(data);

	//set the default scheduling to one seconds
	setFeatures(features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)1000000);

	//get channel pointer
	tmpInt =  getAttributeCache()->getRWPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
                
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
	o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
		
	o_dev_state = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "dev_state");
	o_on = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "on");
	
	o_alarm = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "alarm");

	BC_NORMAL_RUNNIG_PROPERTY
        sequence_number = 0;
	slow_acquisition_idx = 0;
}

    // Aquire the necessary data for the command
/*!
 The acquire handler has the purpose to get all necessary data need the by CC handler.
 \return the mask for the runnign state
 */
void CmdACTDefault::acquireHandler() {
	string desc;
	int err = 0;
	int stato = 0;
	float tmp_float = 0.0F;
	int tmp_uint32 = 0;
	uint64_t tmp_uint64 = 0;
	CMDCU_ << "Acquiring data";
	
	boost::shared_ptr<SharedCacheLockDomain> r_lock = getAttributeCache()->getLockOnCustomAttributeCache();
	r_lock->lock();
	
    if((err = actuator_drv->getPosition(readTyp,&tmp_float))==0){
		*o_position = (double)tmp_float;
    } else {
		LOG_AND_TROW(CMDCUERR_, 1, boost::str( boost::format("Error calling driver on get position readout with code %1%") % err));
	}

		
	if((err = actuator_drv->getAlarms(&tmp_uint64)) == 0){
		*o_alarms = tmp_uint64;
	} else {
		LOG_AND_TROW(CMDCUERR_, 2, boost::str( boost::format("Error calling driver on get alarms readout with code %1%") % err));
	}

	if((err = actuator_drv->getState(&stato, desc)) == 0){
		*o_status_id = stato;
		//update the value and dimension of status channel
		//getAttributeCache()->setOutputAttributeValue("status", (void*)desc.c_str(), (uint32_t)desc.size());
		//the new pointer need to be got (set new size can reallocate the pointer)
		o_status = getAttributeCache()->getRWPtr<char>(DOMAIN_OUTPUT, "status");
		//copy up to 255 and put the termination character
		strncpy(o_status, desc.c_str(), 256);
	} else {
		LOG_AND_TROW(CMDCUERR_, 3, boost::str( boost::format("Error calling driver on get state readout with code %1%") % err));
	}

    CMDCU_ << "position ->" << *o_position;
    CMDCU_ << "alarms ->" << *o_alarms;
    CMDCU_ << "status_id -> " << *o_status_id;
	
	/*
	 * Javascript Interface
	 */
//	*o_on = (*o_status_id & common::actuator::POWER_SUPPLY_STATE_ON) ? 1:0;
	*o_alarm = (*o_alarms!=0)?1:0;

      
    CMDCU_ << "status. -> " << o_status;
    CMDCU_ << "dev_state -> " << *o_dev_state;
    CMDCU_ << "sequence_number -> " << sequence_number;
	
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}
