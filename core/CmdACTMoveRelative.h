/*
 *	CmdACTSetCurrent.h
 *	!CHOAS
 *	Created by Claudio Bisegni.
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

#ifndef __Actuator__CmdMoveRelative__
#define __Actuator__CmdMoveRelative__

#include "AbstractActuatorCommand.h"

#include <bitset>

namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;

namespace driver {
	namespace actuator {
		
		//! Command for change the mode of the actuator
		DEFINE_BATCH_COMMAND_CLASS(CmdACTMoveRelative,AbstractActuatorCommand) {
			
			//double	*o_position_sp; *******commentato **********
			//double	*o_position;    *******commentato **********
			//double *i_speed;              *******commentato **********
			//const uint32_t *axID;*******commentato **********
			
			//const uint32_t	*i_command_timeout;*******commentato **********
			//const double	*__i_delta_setpoint;*******commentato **********
			
			//const double	*__i_setpoint_affinity; *******commentato **********
//::common::actuators::AbstractActuator::readingTypes readTyp;*******commentato **********

			//is the delta to the setpoint that notify the end of command
			//double affinity_set_delta;
                         //bool slow_acquisition_index;
		protected:
			//implemented handler
			//uint8_t implementedHandler();  // ************ Commentato *************
			
			// Set handler
			void setHandler(c_data::CDataWrapper *data);

			//custom acquire method
			void acquireHandler();

			//Correlation and commit phase
			void ccHandler();
			
			//manage the timeout
			bool timeoutHandler();
		};
	}
}


#endif /* defined(__Actuator__CmdSetCurrent__) */
