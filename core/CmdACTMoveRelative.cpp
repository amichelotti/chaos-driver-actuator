/*
 *	CmdACTMoveRelative.cpp
 *	!CHAOS
 *	Created by Alessandro D'Uffizi.
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


#include "CmdACTMoveRelative.h"

#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

#define SCLAPP_ INFO_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "


namespace own =  driver::actuator;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;


BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTMoveRelative,CMD_ACT_MOVE_RELATIVE_ALIAS,
                                                          "Move from current position to an offset (mm)",
                                                          "0cf52f73-55eb-4e13-8712-3d54486040d8")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_MM_OFFSET, "offset in mm",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

// return the implemented handler
uint8_t own::CmdACTMoveRelative::implementedHandler(){
    return	AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
}

void own::CmdACTMoveRelative::setHandler(c_data::CDataWrapper *data) {
    chaos::common::data::RangeValueInfo position_sp_attr_info;
    chaos::common::data::RangeValueInfo attributeInfo;
	AbstractActuatorCommand::setHandler(data);

	double max_position=0,min_position=0;
	int err = 0;
	int state;
        int *tmpInt;
        double position;
	std::string state_str;
	float offset_mm = 0.f;
	chaos::common::data::RangeValueInfo attr_info;
	o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
	o_position_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp");
	i_speed = ( double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");		
	i_command_timeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "command_timeout");
	__i_delta_setpoint = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__delta_setpoint");
	__i_setpoint_affinity = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__setpoint_affinity");
	axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
        tmpInt =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
        
           
        getDeviceDatabase()->getAttributeRangeValueInfo("position_sp", attr_info);
 // REQUIRE MIN MAX SET IN THE MDS

  if (attr_info.maxRange.size()) {
      max_position = atof(attr_info.maxRange.c_str());
    SCLDBG_ << "max_position max=" << max_position;

  } else {
      SCLERR_<< attr_info.name <<":"<< attr_info.maxSize <<";"<<attr_info.maxRange <<":";
           SCLERR_ << "not defined maximum 'position_sp' attribute, quitting command";
          //         BC_END_RUNNIG_PROPERTY;
	  //  return;
  }

  // REQUIRE MIN MAX POSITION IN THE MDS
  if (attr_info.minRange.size()) {
            min_position = atof(attr_info.minRange.c_str());

        SCLDBG_ << "min_position min=" << min_position;
  } else {
                  SCLERR_ << "not defined minimum 'position_sp' attribute, quitting command";
            //       BC_END_RUNNIG_PROPERTY;
	    //return;

  }
        
        
  
 

	//acquire the state readout
	SCLDBG_ << "fetch state readout";
	if((err = actuator_drv->getState(*axID,&state, state_str))) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching state readout with code %1%") % err));
	} else {
		*o_status_id = state;
		//copy up to 255 and put the termination character
		strncpy(o_status, state_str.c_str(), 256);
	}


        if(((*o_status_id)&::common::actuators::ACTUATOR_READY)==0){
            SCLERR_ << boost::str( boost::format("Bad state for moving actuator %1%[%2%]") % o_status % *o_status_id);
	    BC_END_RUNNIG_PROPERTY;
	    return;
        }
	
	SCLDBG_ << "fetch position readout";
        if ((err = actuator_drv->getPosition(*axID,readTyp,&position)) !=0) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
	} else {
                 *o_position = position;
	}

        
       
		
	SCLDBG_ << "check data";
	if(!data ||
	   !data->hasKey(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter not present";
		BC_END_RUNNIG_PROPERTY;
		return;
	}
	if(!data->isDoubleValue(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter is not a Double data type";
		BC_END_RUNNIG_PROPERTY;
		return;
	}
    
    offset_mm = static_cast<float>(data->getDoubleValue(CMD_ACT_MM_OFFSET));
    if(isnan(offset_mm)==true){
        SCLERR_ << "Offset millimeters parameter is not a valid double number (nan?)";
        BC_END_RUNNIG_PROPERTY;
        return;
    }
    

    if((position + offset_mm) > max_position || (position + offset_mm)<min_position){
        	//SCLERR_ << boost::str( boost::format("position %1 'currentSP' \"max_position\":%2% \"min_position\":%3%" ) % (position + offset_mm) % max_position % min_position);
		//BC_END_RUNNIG_PROPERTY;
		//return;
    }
    

    SCLDBG_ << "compute timeout for moving relative = " << offset_mm;
    uint64_t computed_timeout;
	if (*i_speed != 0)
        {
            double ccTim  = offset_mm / *i_speed;
            ccTim*=100;
            ccTim*=10000000;
            computed_timeout = (uint64_t)ccTim;
        }
        else computed_timeout=20000000000;

  	
		
	SCLDBG_ << "Calculated timeout is = " << computed_timeout;
	SCLDBG_ << "o_position_sp is = " << *o_position_sp;
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);
	
        
        //set current set poi into the output channel
	if(*__i_delta_setpoint && (offset_mm < *__i_delta_setpoint)) {
		SCLERR_ << "The offset don't pass delta check of = " << *__i_delta_setpoint << " setpoint point = "<<offset_mm <<" actual position" << *o_position_sp;
		BC_END_RUNNIG_PROPERTY;
		return;
	}

	if(*__i_setpoint_affinity) {
		affinity_set_delta = *__i_setpoint_affinity;
	} else {
		affinity_set_delta = 1;
	}
	SCLDBG_ << "The setpoint affinity value is of +-" << affinity_set_delta << " of millimeters";

	SCLDBG_ << "Move of offset " << offset_mm;
	if((err = actuator_drv->moveRelativeMillimeters(*axID,offset_mm)) != 0) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% setting current") % err));
	}
	
	//assign new position setpoint
	slow_acquisition_index = false;
        
	*o_position_sp =((double)  position+(double)offset_mm);
	actuator_drv->accessor->base_opcode_priority=100;
	setWorkState(true);
	BC_EXEC_RUNNIG_PROPERTY;

}

void own::CmdACTMoveRelative::acquireHandler() {
	int err = 0;
        int state=0;
        double position;
	std::string desc;
        
	std::string state_str;
	SCLDBG_ << "MoveRelative Start Acquire Handler " ;
	//acquire the current readout
	SCLDBG_ << "fetch current readout";
        
        if((err = actuator_drv->getState(*axID,&state, state_str))) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching state readout with code %1%") % err));
	} else {
		*o_status_id = state;
		//copy up to 255 and put the termination character
		strncpy(o_status, state_str.c_str(), 256);
	}
        
	if ((err = actuator_drv->getPosition(*axID,readTyp,&position)) !=0) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
	} else {
                 *o_position = position;
	}
	if((slow_acquisition_index = !slow_acquisition_index)) {
	
	} else {
	    SCLDBG_ << "fetch alarms readout";
		if((err = actuator_drv->getAlarms(*axID,o_alarms,desc))){
			LOG_AND_TROW(SCLERR_, 2, boost::str(boost::format("Error fetching alarms readout with code %1%") % err));
		}
	}
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdACTMoveRelative::ccHandler() {
	//check if we are int the delta of the setpoit to end the command
	double delta_position_reached = std::abs(*o_position_sp - *o_position);
	SCLDBG_ << "Readout: "<< *o_position <<" SetPoint: "<< *o_position_sp <<" Delta to reach: " << delta_position_reached;
	if(delta_position_reached <= affinity_set_delta) 
        {
		uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		//the command is endedn because we have reached the affinitut delta set
		SCLDBG_ << "[metric ]Set point reached with - delta: "<< delta_position_reached <<" sp: "<< *o_position_sp <<" affinity check " << affinity_set_delta << " mm in " << elapsed_msec << " milliseconds";
		BC_END_RUNNIG_PROPERTY;
		setWorkState(false);
        }
        
//	if(*o_alarms) {
//		SCLERR_ << "We got alarms on actuator so we end the command";
//		BC_END_RUNNIG_PROPERTY;
//		setWorkState(false);
//	}
}

bool own::CmdACTMoveRelative::timeoutHandler() {
	double delta_position_reached = std::abs(*o_position_sp - *o_position);
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	//move the state machine on fault
	setWorkState(false);
	actuator_drv->accessor->base_opcode_priority=50;
        SCLDBG_ << "ALEDEBUG delta position reached " << delta_position_reached << " affinity_set_delta " << affinity_set_delta ;
        
	if(delta_position_reached <= affinity_set_delta) {
		uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		SCLDBG_ << "[metric] Setpoint reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
		//the command is endedn because we have reached the affinitut delta set
		BC_END_RUNNIG_PROPERTY;
	}else {
		SCLERR_ << "[metric] Setpoint not reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
		BC_END_RUNNIG_PROPERTY;
		//BC_FAULT_RUNNIG_PROPERTY;
	}
	return false;
}
