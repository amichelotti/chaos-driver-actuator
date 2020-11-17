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

void ChaosActuatorDD::driverDeinit()  {
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
    if(motor==NULL){
      ACERR<<"motor low level driver NULL, executing opcode:"<<cmd->opcode;
      return result;
    }
    switch(cmd->opcode){
        case OP_INIT:
             ACDBG<< "Initializing";
             out->result = motor->init(in->str);
            break;
            
        case OP_CONFIGAXIS:
             ACDBG<< "Configuring " << in->str;
             out->result = motor->configAxis(in->str);
            break;
        case OP_DEINIT:
             ACDBG<< "Deinitializing";
             out->result = motor->deinit(in->axis);
            break;
          
        case OP_GET_POSITION:
            out->result=motor->getPosition(in->axis,(::common::actuators::AbstractActuator::readingTypes)in->ivalue,&out->fvalue0);
          //  ACDBG<<"Got Position :"<< out->fvalue0;
            break; 
            
        case OP_RESET_ALARMS:
            ACDBG<<"Reset alarms to:"<<in->alarm_mask << std::endl;
            out->result = motor->resetAlarms(in->axis,in->alarm_mask);
            break;
        case OP_HARD_RESET:
            ACDBG<<"HARD Reset in axis "<< in->axis << std::endl;
            out->result = motor->hardreset(in->axis,in->ivalue);
            break;
        case OP_GET_ALARMS: {
	    std::string desc;
            out->result = motor->getAlarms(in->axis,&out->alarm_mask,desc);
            strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
        //    ACDBG<<"Got alarms to: "<<out->alarm_mask << desc;
            break;
           }
            
         case OP_MOVE_RELATIVE_MM:
            out->result = motor->moveRelative(in->axis,in->fvalue0);
            ACDBG<<"Move relative result:"<<out->result;
            break;
        
        case OP_MOVE_ABSOLUTE_MM:
            out->result = motor->moveAbsolute(in->axis,in->fvalue0);
            ACDBG<<"Moved Absolute result:"<<out->result;
            break;
       
        case OP_STOP_MOTION:
            ACDBG<<"Before Stop Motion :" << std::endl;
            out->result = motor->stopMotion(in->axis);
            ACDBG<<"Stop Motion, result:"<< out->result;
            break;
        
        case OP_HOMING:
            out->result = motor->homing(in->axis, (::common::actuators::AbstractActuator::homingType)in->ivalue );
            ACDBG<<"Set homing, homing type: "<< in->ivalue << "result is " << out->result;
            break;
            
            
        case OP_POWERON:
            out->result = motor->poweron(in->axis,in->ivalue);
            ACDBG<<"Set Power" << in->ivalue <<" , result:"<< out->result;
            break;
            
         case OP_GET_STATE:{
            std::string desc;
            out->result = motor->getState(in->axis,&out->ivalue,desc);
            strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
        //    ACDBG<<"Got State: "<<out->ivalue<<" \""<<desc;
            break;
        }
            
        case OP_GET_SWVERSION:{
            std::string ver;
            out->result = motor->getSWVersion(in->axis,ver);
            ACDBG<<"Got HW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }
        case OP_GET_HWVERSION:{
            std::string ver;
            out->result = motor->getHWVersion(in->axis,ver);
            ACDBG<<"Got SW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;

            break;
        }            
        case OP_LISTPARAMETERS:{
        std::string dataset;
	out->result=motor->listParameters(dataset);
	//ACDBG << "ALEDEBUG: Received dataset " <<  dataset;
        strncpy(out->str,dataset.c_str(),JSON_MAX_SIZE);
	break;
	}
        case OP_SETPARAMETER:{
            char* SS0=strdup(in->str);
	char* SS1=strdup(in->str2);
	out->result=motor->setParameter(in->axis,SS0,SS1);
	
	ACDBG << "ALEDEBUG: Sending Set " << SS1 <<"  on Parameter " << SS0 <<"axis " <<in->axis;
	ACDBG << "result " << out->result ;
	free(SS0);
	free(SS1);
	break;
	}
        case OP_GETPARAMETER: {
        	std::string tempString;
        	out->result=motor->getParameter(in->axis,in->str,tempString);

        	strncpy(out->str,tempString.c_str(),JSON_MAX_SIZE);
        	ACDBG << "ALEDEBUG (DD) asked for " << in->str << " received " <<out->str;
        	break;
        }
            
        case OP_GET_FEATURE:{
            uint64_t feat=motor->getFeatures();
            out->alarm_mask=feat;
       //     ACDBG<<"Got Features:"<<feat;
        }
            break;
        
        default:
            ACERR<<"Opcode not supported:"<<cmd->opcode;
    }
    return result;
}





