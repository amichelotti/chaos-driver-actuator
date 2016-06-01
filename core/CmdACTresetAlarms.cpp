/*
CmdACTresetAlarms.cpp
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
#include "CmdACTresetAlarms.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTresetAlarms) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTresetAlarms,CMD_ACT_RESETALARMS_ALIAS,
			"Reset the alarms on the actuator",
			"23db61e1-919f-48d8-b8a9-e9dc44a04688")
BATCH_COMMAND_ADD_INT64_PARAM(CMD_ACT_ALRM,"alarm mask",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
uint8_t own::CmdACTresetAlarms::implementedHandler(){
	return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
}
// empty set handler
void own::CmdACTresetAlarms::setHandler(c_data::CDataWrapper *data) {
	AbstractActuatorCommand::setHandler(data);
}
// empty acquire handler
void own::CmdACTresetAlarms::acquireHandler() {
}
// empty correlation handler
void own::CmdACTresetAlarms::ccHandler() {
}
// empty timeout handler
bool own::CmdACTresetAlarms::timeoutHandler() {
}
