/*
 *	CmdACTDefault.h
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

#ifndef __Actuator__CmdACTDefault__
#define __Actuator__CmdACTDefault__

#include "AbstractActuatorCommand.h"


namespace c_data = chaos::common::data;
namespace ccc_slow_command = chaos::cu::control_manager::slow_command;

namespace driver {
	namespace actuator {
		
		DEFINE_BATCH_COMMAND_CLASS(CmdACTDefault,AbstractActuatorCommand) {
			uint64_t        OutOfSetWarningTimestamp;
                        bool            OutOfSetWarningStatus;
			unsigned int	slow_acquisition_idx;
			//const uint32_t *axID;
            //uint64_t	*o_dev_state;
                      //::common::actuators::AbstractActuator::readingTypes   readTyp;
                        //double          *o_position;
                        double          *o_positionEnc;
			double		*o_acceleration;
                        const double          *positionTHR;
                        const double          *positionTHR_TIM;
                        const double          *posRes;
			int32_t         *o_warning;
			int32_t		*o_on;
			int32_t		*o_alarm;
                        
                            //Last values for choose to push or not
                        int32_t lastState;
                        double lastPosition;
                        uint64_t lastAlarms;
						
			int32_t alreadyLogged;
  			int32_t BackupAxID;
			c_data::CDataWrapper *BackupData;
                        

		protected:
			// return the implemented handler
			uint8_t implementedHandler();
			
			// Start the command execution
			void setHandler(c_data::CDataWrapper *data);
			
			// Aquire the necessary data for the command
			/*!
			 The acquire handler has the purpose to get all necessary data need the by CC handler.
			 \return the mask for the runnign state
			 */
			void acquireHandler();
		public:
			CmdACTDefault();
			~CmdACTDefault();
		};
		
	}
}


#endif /* defined(__Actuator__CmdACTDefault__) */
