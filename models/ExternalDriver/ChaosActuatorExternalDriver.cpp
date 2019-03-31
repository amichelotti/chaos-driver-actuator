/*
 *	ChaosActuatorExternalDriver.cpp
 *
 *	!CHAOS
 *	Created by  D'Uffizi.
 *
 *    	Copyright 25/07/2017 INFN, National Institute of Nuclear Physics
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

#include "ChaosActuatorExternalDriver.h"
using namespace chaos::driver::actuator;
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ChaosActuatorRemoteServerDriver, 1.0.0, chaos::driver::actuator::ChaosActuatorRemoteServerDriver)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ChaosActuatorRemoteClientDriver, 1.0.0, chaos::driver::actuator::ChaosActuatorRemoteClientDriver)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION
