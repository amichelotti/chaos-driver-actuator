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
using namespace chaos::cu::control_manager;

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
         
        lastState=-1;
        lastPosition=-1;
        lastAlarms=0;
        
        
        
	CMDCU_ << "Set Handler";
	AbstractActuatorCommand::setHandler(data);

	CMDCU_ << "After parental Set Handler";
	//set the default scheduling to one seconds
	setFeatures(features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)1000000);
	//get channel pointer
        positionTHR= getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "positionWarningTHR");
        positionTHR_TIM=getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "positionWarningTHR_Timeout");
	
        OutOfSetWarningStatus=false;
	BC_NORMAL_RUNNING_PROPERTY
       
        
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
        
        
        //acquire the current readout
        AbstractActuatorCommand::acquireHandler();
	
	
//	tmpInt =  getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
//        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
	
	pos_sp = i_position ;//getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");

        
	if (*o_alarms)
            DPRINT("alarm description is %s",o_alarm_str);
		
        //check out of set warning
        if ((*positionTHR) && (*positionTHR_TIM))
        {
            if (abs(*pos_sp - *o_position) > *positionTHR)
            {
                time_t now=time(NULL);
                if (!OutOfSetWarningStatus)
                {
                    OutOfSetWarningStatus=true;
                    OutOfSetWarningTimestamp=now;
                }
                else
                {
                    if ((now-OutOfSetWarningTimestamp) > *positionTHR_TIM)
                    {
                        CMDCUERR_ << "WARNING OUT OF SET " << *o_position << " ";
                        setStateVariableSeverity(StateVariableTypeAlarmDEV,"position_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
                        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,"The position set point has drifted out the defined threshold" );     
                  
                    }
                }
                
            }
            else
            {
                OutOfSetWarningStatus=false;
                setStateVariableSeverity(StateVariableTypeAlarmDEV,"position_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
            }
        }

    CMDCU_ << "Axis ID ->" << (int) *axID;
    CMDCU_ << "Reading Type ->" << (int) readTyp;
    CMDCU_ << "position_sp ->" << *pos_sp;
    CMDCU_ << "position ->" << *o_position;
    //CMDCU_ << "position by encoder ->" << EncRead;
    CMDCU_ << "alarms ->" << *o_alarms;
    CMDCU_ << "alarm desc -> " << o_alarm_str;
    CMDCU_ << "status_id -> " << *o_status_id;
    CMDCU_ << "status desc -> " << o_status_str;
	
	//force output dataset as changed
    if (  (lastState!=*o_status_id) || 
           ((abs(lastPosition - *o_position)> *p_resolution)) ||
            (lastAlarms!=*o_alarms) 
            )
    {
	getAttributeCache()->setOutputDomainAsChanged();
        lastState=*o_status_id;
        lastPosition=*o_position;
        lastAlarms=*o_alarms;
    }
}
