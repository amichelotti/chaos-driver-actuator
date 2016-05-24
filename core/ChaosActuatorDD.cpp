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

//default destructor
ChaosActuatorDD::~ChaosActuatorDD() {
	
}

void ChaosActuatorDD::driverDeinit() throw(chaos::CException) {
    if(motor){
        delete motor;
    }
    motor = NULL;
}


cu_driver::MsgManagmentResultType::MsgManagmentResult ChaosActuatorDD::execOpcode(cu_driver::DrvMsgPtr cmd){
//        boost::mutex::scoped_lock lock(io_mux);

    cu_driver::MsgManagmentResultType::MsgManagmentResult result = cu_driver::MsgManagmentResultType::MMR_EXECUTED;
    actuator_iparams_t *in = (actuator_iparams_t *)cmd->inputData;
    actuator_oparams_t *out = (actuator_oparams_t *)cmd->resultData;

    switch(cmd->opcode){
        case OP_INIT:
             ACDBG<< "Initializing";
             out->result = motor->init(in->str);
            break;
            
        case OP_DEINIT:
             ACDBG<< "Deinitializing";
             out->result = motor->deinit();
            break;
        case OP_SET_TIMEOUT:
            out->result= motor->setTimeout(in->timeout);
            ACDBG<<"Set timeout to:"<<in->timeout;
            break;
        
        case OP_GET_TIMEOUT:
            out->result= motor->getTimeout(&out->alarm_mask);
            ACDBG<<"Got timeout to: "<<out->alarm_mask;
            break;
            
         case OP_SET_SPEED:
            out->result= motor->setSpeed(in->fvalue0);
            ACDBG<<"Set speed to:"<<in->fvalue0;
            break;
            
        case OP_SET_ACCELERATION:
            out->result= motor->setAcceleration(in->fvalue0);
            ACDBG<<"Set acceleration to:"<<in->fvalue0;
            break;
            
        case OP_SET_ADDITIVE:
            motor->setAdditive(in->ivalue);
            ACDBG<<"Set additive to:"<<in->ivalue;
            break;
        
        case OP_SET_REFERENCE:
            out->result= motor->setReferenceBase(in->ivalue);
            ACDBG<<"Set referenceBase to:"<<in->ivalue;
            break;
            
        case OP_SET_MOVEMENT:
            out->result= motor->setMovement(in->ivalue);
            ACDBG<<"Set Movement to:"<<in->ivalue;
            break;    
            
        case OP_SET_ACCELERATION_HOM:
            out->result= motor->setAccelerationHoming(in->fvalue0);
            ACDBG<<"Set acceleration for homing to:"<<in->fvalue0;
            break;
            
        case OP_SET_ADDITIVE_HOM:
            motor->setAdditiveHoming(in->ivalue);
            ACDBG<<"Set additive for homing to:"<<in->ivalue;
            break;
        
        case OP_SET_REFERENCE_HOM:
            out->result= motor->setReferenceBaseHoming(in->ivalue);
            ACDBG<<"Set referenceBase for homing to:"<<in->ivalue;
            break;
            
        case OP_SET_MOVEMENT_HOM:
            out->result= motor->setMovementHoming(in->ivalue);
            ACDBG<<"Set Movement for homing to:"<<in->ivalue;
            break;    
            
        case OP_GET_POSITION:
            out->result=motor->getPosition((::common::actuators::AbstractActuator::readingTypes)in->ivalue,&out->fvalue0);
            ACDBG<<"Got Position :"<< out->fvalue0;
            break; 
            
        case OP_RESET_ALARMS:
            ACDBG<<"Reset alarms to:"<<in->alarm_mask;
            out->result = motor->resetAlarms(in->alarm_mask);
            break;
        case OP_GET_ALARMS: {
	    std::string desc;
            out->result = motor->getAlarms(&out->alarm_mask,desc);
            strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
            ACDBG<<"Got alarms to:"<<out->alarm_mask << desc;
            break;
           }
            
         case OP_MOVE_RELATIVE_MM:
            out->result = motor->moveRelativeMillimeters(in->fvalue0);
            ACDBG<<"Move relative result:"<<out->result;
            break;
        
        case OP_MOVE_ABSOLUTE_MM:
            out->result = motor->moveAbsoluteMillimeters(in->fvalue0);
            ACDBG<<"Moved Absolute result:"<<out->result;
            break;
       
        case OP_STOP_MOTION:
            out->result = motor->stopMotion();
            ACDBG<<"Stop Motion, result:"<< out->result;
            break;
        
        case OP_HOMING:
            out->result = motor->homing( (::common::actuators::AbstractActuator::homingType)in->ivalue );
            ACDBG<<"Set homing, homing type: "<< in->ivalue << "result is " << out->result;
            break;
            
            
        case OP_POWERON:
            out->result = motor->poweron();
            ACDBG<<"Set Power on, result:"<< out->result;
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
            out->result = motor->getSWVersion(ver);
            ACDBG<<"Got HW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }
        case OP_GET_HWVERSION:{
            std::string ver;
            out->result = motor->getHWVersion(ver);
            ACDBG<<"Got SW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }            
       
            
        case OP_GET_FEATURE:{
            uint64_t feat=motor->getFeatures();
            out->alarm_mask=feat;
            ACDBG<<"Got Features:"<<feat;
        }
            break;
        
        default:
            ACERR<<"Opcode not supported:"<<cmd->opcode;
    }
    return result;
}





