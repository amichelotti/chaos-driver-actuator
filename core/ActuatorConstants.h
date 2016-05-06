/*
 *	ActuatorCostants.h
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


#ifndef Actuator_ActuatorCostants_h
#define Actuator_ActuatorCostants_h

namespace driver {
	namespace actuator {
		
        #define TROW_ERROR(n,m,d) throw chaos::CException(n, m, d);
		
        #define LOG_TAIL(n) "[" << #n << "] - " << getDeviceID() << " - [" << getUID() << "] - "
        
		//! The alias of the default command of the type
		const char * const CMD_ACT_DEFAULT_ALIAS = "default";
		
		//! The alias of the "mode" command of the type
		const char * const CMD_ACT_MODE_ALIAS = "mode";
		//! The alias of the parameter of the type {0-to stadby, 1-to operational}
		const char * const CMD_ACT_MOVE_RELATIVE = "move_rela";
		
		//! The alias of the "reset" command of the type
		const char * const CMD_ACT_RESET_ALIAS = "rset";

		//! The alias of the "stop motion" command of the type
		const char * const CMD_ACT_STOPMOTION_ALIAS = "stop";
		//! The alias of the slope up parameter
		const char * const CMD_ACT_SET_SLOPE_UP = "sslp_up";
		//! The alias of the slope down parameter
		const char * const CMD_ACT_SET_SLOPE_DOWN = "sslp_down";
		
		//! The alias of the "move rel" command of the type
		const char * const CMD_ACT_MOVE_RELATIVE_ALIAS = "mov_rel";
		//! The alias of the "move absolute" command of the type
		const char * const CMD_ACT_MOVE_ABSOLUTE_ALIAS = "mov_abs";
		const char * const CMD_ACT_MOVE_ABSOLUTE = "move_absolute";
		//! The alias of the current to set
		const char * const CMD_ACT_MM_OFFSET = "offset_mm";
        
            //! The alias of the "set polarity" command
		const char * const CMD_ACT_SET_POLARITY_ALIAS = "pola";
            //! the type of polarity
		const char * const CMD_ACT_SET_POLARITY_VALUE = "pola_value";
                
        const char * const CMD_ACT_CALIBRATE = "calibrate";
        const char * const CMD_ACT_CALIBRATE_FROM = "from";
        const char * const CMD_ACT_CALIBRATE_TO = "to";
        const char * const CMD_ACT_CALIBRATE_STEACT = "steps";

        #define DEFAULT_COMMAND_TIMEOUT_MS   10000
        #define DEFAULT_RAMP_TIME_OFFSET_MS  10000

	}
}
#endif
