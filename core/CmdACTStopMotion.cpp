/*
CmdACTStopMotion.cpp
!CHAOS
Created by CUGenerator

Copyright 2013 INFN, National Institute of Nuclear Physics
Licensed under the Apache License, Version 2.0 (the "License")
you may not use this file except in compliance with the License.
      You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/
#include "CmdACTStopMotion.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTStopMotion) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTStopMotion,CMD_ACT_STOPMOTION_ALIAS,
			"Stop the Motion of the Actuator, if any",
			"63768ac0-11dc-11e6-8629-233988a40683")
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
uint8_t own::CmdACTStopMotion::implementedHandler(){
	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
}
// empty set handler
void own::CmdACTStopMotion::setHandler(c_data::CDataWrapper *data) {
	int err=0;
AbstractActuatorCommand::setHandler(data);
	axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");

	SCLDBG_ << "Stop Motion " ;
	if((err = actuator_drv->stopMotion(*axID)) != 0) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error %1% stopping motor") % err));
	}
actuator_drv->accessor->base_opcode_priority=100;
	setWorkState(true);
	BC_EXEC_RUNNING_PROPERTY;
}
// empty acquire handler
void own::CmdACTStopMotion::acquireHandler() {
	int err = 0;
	int state = 0;
	std::string desc;
	if((err = actuator_drv->getState(*axID,&state, desc))) {
		LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching state readout with code %1%") % err));
	} else {
		*o_status_id = state;
		//copy up to 255 and put the termination character
		strncpy(o_status_str, desc.c_str(), 256);
	}

	if((slow_acquisition_index = !slow_acquisition_index)) {
	
	} else {
	    SCLDBG_ << "fetch alarms readout";
		if((err = actuator_drv->getAlarms(*axID,o_alarms,desc))){
			LOG_AND_TROW(SCLERR_, 2, boost::str(boost::format("Error fetching alarms readout with code %1%") % err));
		}
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
	}
}
// empty correlation handler
void own::CmdACTStopMotion::ccHandler() {
	int err=0;
	double position;
        uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	//the command is endedn because we have reached the affinitut delta set
	SCLDBG_ << "cc handler Stop motion " << ((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION) ;
        DPRINT(" :   %x %x",*o_status_id, ::common::actuators::ACTUATOR_INMOTION);
	if (((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION)==0)
	{
	     SCLDBG_ << "[metric ]:Motor Stopped  in " << elapsed_msec << " milliseconds";
	     setWorkState(false);
	     BC_END_RUNNING_PROPERTY;
	}
	if (*o_alarms) {
		SCLERR_ << "We got alarms on actuator so we end the command";
		setWorkState(false);
		BC_END_RUNNING_PROPERTY;
	}
}
// empty timeout handler
bool own::CmdACTStopMotion::timeoutHandler() {
	uint64_t elapsed_msec;
	setWorkState(false);
	actuator_drv->accessor->base_opcode_priority=50;
	if ((o_status_str && ::common::actuators::ACTUATOR_INMOTION)==0)
        {
	   uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	   SCLDBG_ << "[metric] Motor Stopped on timeout in " << elapsed_msec << " milliseconds";
	   //the command is endedn because we have reached the affinitut delta set
	   BC_END_RUNNING_PROPERTY;
	}else {
           SCLERR_ << "[metric] Motor not stopped before timeout of " << elapsed_msec << " milliseconds";
	   BC_FAULT_RUNNING_PROPERTY;
	}
	return false;
	
}
