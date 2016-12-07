/*
 *	AbstractActuatorCommand.h
 *	!CHOAS
 *	Created by Claudio Bisegni.
 *      Adapted for actuators by Alessandro D'Uffizi
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

#ifndef __Actuator__AbstractActuatorCommand__
#define __Actuator__AbstractActuatorCommand__

#include "ActuatorConstants.h"

#include <driver/actuator/core/ChaosActuatorInterface.h>

#include <chaos/cu_toolkit/control_manager/slow_command/SlowCommand.h>

namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;

namespace driver {
	namespace actuator {
		
		class AbstractActuatorCommand : public ccc_slow_command::SlowCommand {

		public:
		  AbstractActuatorCommand();
		  ~AbstractActuatorCommand();
		protected:
                        int32_t		*o_status_id;
			char            *o_status_str;
                        uint64_t	*o_alarms_id;
			char		*o_alarm_str;
                        double          *o_position;
                        ::common::actuators::AbstractActuator::readingTypes readTyp;
                        
                        double	*o_position_sp; // ********* AGGIUNTO ************
                        double *i_speed; // ********* AGGIUNTO ************
                        const uint32_t	*i_command_timeout;// ********* AGGIUNTO ************
                        const double	*__i_delta_setpoint;// ********* AGGIUNTO ************
                        const double	*__i_setpoint_affinity;// ********* AGGIUNTO ************
                        
                        const double*p_minimumWorkingValue,*p_maximumWorkingValue;  // ********* AGGIUNTO ************
                         
                        const uint32_t *axID;       // ********* AGGIUNTO axID ************
                        const bool      *s_bypass;

			//reference of the chaos bastraction ofactuator driver
			chaos::driver::actuator::ChaosActuatorInterface *actuator_drv;
			
			//implemented handler
			uint8_t implementedHandler();

                        void acquireHandler(); // ******** Aggiunto qui!!! ********
			void ccHandler();
			
			// set the data fr the command
			void setHandler(c_data::CDataWrapper *data);

			void getState(int32_t axisID,int& current_state, std::string& current_state_str);
			
			void setWorkState(bool working);
		};
	}
}

#endif /* defined(__Actuator__AbstractActuatorCommand__) */
