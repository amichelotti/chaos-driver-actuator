/*
 *	Actuator base 
 *	!CHAOS
 *	Created by Alessandro D'Uffizi
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
#include "ChaosActuatorDD.h"

#include <string>
#include <boost/regex.hpp>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include "driver/actuator/core/ChaosActuatorInterface.h"

#define ACLAPP		LAPP_ << "[ActuatorDD] "
#define ACDBG		LDBG_ << "[ActuatorDD] "
#define ACERR		LERR_ << "[ActuatorDD] "

using namespace chaos::driver::actuator;
//default constructor definition
DEFAULT_CU_DRIVER_PLUGIN_CONSTRUCTOR_WITH_NS(chaos::driver::actuator, ChaosActuatorDD) {
    motor = NULL;
	
}

//default descrutcor
ChaosActuatorDD::~ChaosActuatorDD() {
	
}

void ChaosActuatorDD::driverDeinit() throw(chaos::CException) {
    if(motor){
        delete motor;
    }
    motor = NULL;
}


cu_driver::MsgManagmentResultType::MsgManagmentResult ChaosActuatorDD::execOpcode(cu_driver::DrvMsgPtr cmd){
        boost::mutex::scoped_lock lock(io_mux);

    cu_driver::MsgManagmentResultType::MsgManagmentResult result = cu_driver::MsgManagmentResultType::MMR_EXECUTED;
    actuator_iparams_t *in = (actuator_iparams_t *)cmd->inputData;
    actuator_oparams_t *out = (actuator_oparams_t *)cmd->resultData;

    switch(cmd->opcode){
        case OP_INIT:
             ACDBG<< "Initializing";
             out->result = motor->init();
            break;
            
        case OP_DEINIT:
             ACDBG<< "Deinitializing";
             out->result = motor->deinit();
            break;
            
        case OP_SET_SP:
            ACDBG<< "Set Current SP to:"<<in->fvalue0<<" timeo:"<<in->timeout;
            out->result = motor->setCurrentSP(in->fvalue0,in->timeout);
            break;
            
       
            
        case OP_GET_SP: // get current set point
            out->result = motor->getCurrentSP(&out->fvalue0,in->timeout);
            ACDBG<< "Got Current SP "<<out->fvalue0;
            break;
        case OP_RESET_ALARMS:
            ACDBG<<"Reset alarms to:"<<in->alarm_mask<<" timeout:"<<in->timeout;
            out->result = motor->resetAlarms(in->alarm_mask,in->timeout);
            break;
        case OP_GET_ALARMS:
            out->result = motor->getAlarms(&out->alarm_mask,in->timeout);
            ACDBG<<"Got alarms to:"<<out->alarm_mask<<" timeout:"<<in->timeout;
            break;
            
        case OP_GET_FEATURE:{
            uint64_t feat=motor->getFeatures();
            out->alarm_mask=feat;
            ACDBG<<"Got Features:"<<feat;
        }
            break;
        
        case OP_POWERON:
            ACDBG<<"Poweron "<<" timeout:"<<in->timeout;
            out->result = motor->poweron(in->timeout);
            break;
        case OP_GET_STATE:{
            std::string desc;
            out->result = motor->getState(&out->ivalue,desc);
            strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
            ACDBG<<"Got State: "<<out->ivalue<<" \""<<desc;
            break;
        }
        case OP_GET_SWVERSION:{
            std::string ver;
            out->result = motor->getSWVersion(ver,in->timeout);
            ACDBG<<"Got HW Version:\""<<ver<<"\" timeout:"<<in->timeout;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }
        case OP_GET_HWVERSION:{
            std::string ver;
            out->result = motor->getHWVersion(ver,in->timeout);
            ACDBG<<"Got SW Version:\""<<ver<<"\" timeout:"<<in->timeout;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }
                
        case OP_GET_ALARM_DESC:
            out->result = motor->getAlarmDesc(&out->alarm_mask);
            ACDBG<<"Got Alarm maxk "<<out->alarm_mask;

            break;
        default:
            ACERR<<"Opcode not supported:"<<cmd->opcode;
    }
    return result;
}





