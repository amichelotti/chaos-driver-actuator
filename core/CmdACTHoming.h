/*
CmdACTHoming.h
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
#ifndef __Actuator__CmdACTHoming__
#define __Actuator__CmdACTHoming__
#include "AbstractActuatorCommand.h"
#include "CmdACTStopMotion.h"

#include <bitset>
namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;
namespace driver {
	namespace actuator {
		 DEFINE_BATCH_COMMAND_CLASS(CmdACTHoming,AbstractActuatorCommand) {
			int  homResult;
			int32_t homingTypeVar;  // ok
		 	uint64_t scheduleTime;	
			//double	*o_position;
			//const uint32_t *axID;

			//implemented handler
			//uint8_t implementedHandler();
			//initial set handler
			void setHandler(c_data::CDataWrapper *data);
			//custom acquire handler
			void acquireHandler();
			//correlation and commit handler
			void ccHandler();
			//manage the timeout 
			bool timeoutHandler();
			};
		}
	}
#endif
