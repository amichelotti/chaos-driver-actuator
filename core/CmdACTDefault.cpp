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
	
        const int *tmpInt;
	CMDCU_ << "Set Handler";
	AbstractActuatorCommand::setHandler(data);
	CMDCU_ << "After parental Set Handler";

	//set the default scheduling to one seconds
	setFeatures(features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)1000000);

	//get channel pointer
	tmpInt =  getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
	CMDCU_ << "Set Handler readingType is " << *tmpInt;
                
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
	o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
		
	
	o_alarm = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "alarms");

	BC_NORMAL_RUNNIG_PROPERTY
        sequence_number = 0;
	slow_acquisition_idx = 0;
}

    // Aquire the necessary data for the command
/*!
 The acquire handler has the purpose to get all necessary data need the by CC handler.
 \return the mask for the running state
 */
void CmdACTDefault::acquireHandler() {
	std::string desc;
	int err = 0;
	int32_t stato = -1;
	const int32_t *tmpInt;
	double tmp_float = 0.0F;
	int tmp_uint32 = 0;
	uint64_t tmp_uint64 = -1;
        double EncRead;
        double *pos_sp;
	CMDCU_ << "Acquiring data";
	
	tmpInt =  getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
	
	pos_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp");
    if((err = actuator_drv->getPosition(readTyp,&tmp_float))==0){
		*o_position = tmp_float;
    } else {
		LOG_AND_TROW(CMDCUERR_, 1, boost::str( boost::format("Error calling driver on get position readout with code %1%") % err));
	}
    if((err = actuator_drv->getPosition(::common::actuators::AbstractActuator::READ_ENCODER,&tmp_float))==0){
		EncRead = tmp_float;
    } else {
		LOG_AND_TROW(CMDCUERR_, 1, boost::str( boost::format("Error calling driver on get position readout with code %1%") % err));
	}
        
        
	tmp_uint64=0;		
	if((err = actuator_drv->getAlarms(&tmp_uint64,desc)) == 0){
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
		//strncpy(o_status, desc.c_str(), 256);
	} else {
		LOG_AND_TROW(CMDCUERR_, 3, boost::str( boost::format("Error calling driver on get state readout with code %1%") % err));
	}

    CMDCU_ << "Reading Type ->" << (int) readTyp;
    CMDCU_ << "position_sp ->" << *pos_sp;
    CMDCU_ << "position ->" << *o_position;
    CMDCU_ << "position by encoder ->" << EncRead;
    CMDCU_ << "alarms ->" << *o_alarms;
    CMDCU_ << "status_id -> " << *o_status_id;
	
//	*o_alarm = (*o_alarms!=0)?1:0;

      
	
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}
