/*



 *	
 *	!CHAOS
 *	Created by Alessandro D'Uffizi
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
#include "TechnosoftMotor.h"
#include <common/actuators/models/technosoft/ActuatorTechnoSoft.h>
#include <string>
#include <boost/regex.hpp>

#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are making a plugin

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(TechnosoftMotor, 1.0.0, chaos::driver::actuator::TechnosoftMotor)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::TechnosoftMotor, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(chaos::driver::actuator::TechnosoftMotor)
CLOSE_REGISTER_PLUGIN


//default constructor definition
chaos::driver::actuator::TechnosoftMotor::TechnosoftMotor() {
    motor = NULL;
   
	
}

//default destructor
chaos::driver::actuator::TechnosoftMotor::~TechnosoftMotor() {
	
}
#ifdef CHAOS
void chaos::driver::actuator::TechnosoftMotor::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException) {
     int ret;
    PSLAPP << "Init  driver initialization with json " <<json.getJSONString().c_str();
    if(motor)
    {
          throw chaos::CException(1, "Already Initialized", "TechnosoftMotor::driverInit");
    }
    motor = new ::common::actuators::models::ActuatorTechnoSoft();
    
    if(motor==NULL)
    {
      throw chaos::CException(1, "Cannot allocate resources for TechnosoftMotor", "TechnosoftMotor::driverInit");
    }
    // DA CAMBIARE DOVREBBE ESSERE ALLOCATO DA ACTUATORTECHNOSOFT
    // POSSIBILMENTE SMART POINTER
    motor->jsonConfiguration.setSerializedJsonData(json.getCompliantJSONString().c_str());

   
      if (  (ret=motor->init(NULL)) < 0) 
      {
	PSLAPP<<"Init Failed! ret:"<<ret<<std::endl;
	throw chaos::CException(1, "Bad parameters for TechnosoftMotor","TechnosoftMotor::driverInit");
      } 
      else 
      {
	PSLAPP<<"Init Done" <<std::endl;
      }
    
   
}
#endif


void chaos::driver::actuator::TechnosoftMotor::driverInit(const char *initParameter) throw(chaos::CException) {
    //check the input parameter
	boost::smatch match;
	std::string inputStr = initParameter;
	PSLAPP << "Init driver initialization string:\""<<initParameter<<"\""<<std::endl;
    if(motor){
          throw chaos::CException(1, "Already Initialized", "TechnosoftMotor::driverInit");
    }
    motor = new ::common::actuators::models::ActuatorTechnoSoft();
    
    if(motor==NULL)
    {
      throw chaos::CException(1, "Cannot allocate resources for TechnosoftMotor", "TechnosoftMotor::driverInit");
    } 
    else 
    { 
      // DA CAMBIARE DOVREBBE ESSERE ALLOCATO DA ACTUATORTECHNOSOFT
     
      int ret;
      if (  (ret=motor->init((void*)initParameter)) < 0) 
      {
	PSLAPP<<"Init Failed of:" <<initParameter<<" ret:"<<ret<<std::endl;
	throw chaos::CException(1, "Bad parameters for TechnosoftMotor","TechnosoftMotor::driverInit");
      } 
      else 
      {
	PSLAPP<<"Init Done" <<std::endl;
      }
    }
    
}

