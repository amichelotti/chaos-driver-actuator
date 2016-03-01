/*
 *	Hazemeyer AL250 Device Driver
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

#include <string>
#include <boost/regex.hpp>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>

// initialization format is <ACTUATOR TYPE>:'<INITALISATION PARAMETERS>'
static const boost::regex power_supply_init_match("(\\w+):(.+)");

// initialisation format for ocem <serialstring>:<slaveid>
static const boost::regex power_supply_hazemeyer_init_match("([\\w\\/,]+):(\\d+)");

//GET_PLUGIN_CLASS_DEFINITION
//we need only to define the driver because we don't are makeing a plugin

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(C_TechnosoftMotor, 1.0.0, chaos::driver::actuator::C_TechnosoftMotor)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::C_TechnosoftMotor, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(chaos::driver::actuator::C_TechnosoftMotor)
CLOSE_REGISTER_PLUGIN


//default constructor definition
chaos::driver::actuator::C_TechnosoftMotor::C_TechnosoftMotor() {
    motor = NULL;
   
	
}

//default descrutcor
chaos::driver::actuator::C_TechnosoftMotor::~C_TechnosoftMotor() {
	
}

void chaos::driver::actuator::C_TechnosoftMotor::driverInit(const char *initParameter) throw(chaos::CException) {
    //check the input parameter
	boost::smatch match;
	std::string inputStr = initParameter;
	PSLAPP << "Init  driver initialisation string:\""<<initParameter<<"\""<<std::endl;
    if(motor){
          throw chaos::CException(1, "Already Initialised", "C_TechnosoftMotor::driverInit");
    }
    if(regex_match(inputStr, match, power_supply_init_match, boost::match_extra)){
        std::string actuatorType=match[1];
        std::string initString=match[2];
        if(actuatorType=="TechnosoftActuator"){
            if(regex_match(initString, match, power_supply_hazemeyer_init_match, boost::match_extra)){
                std::string dev=match[1];
                std::string slaveid=match[2];
                PSLAPP<<"Allocating HazemeyerAL250 device \""<<slaveid<<"\""<<" on dev:\""<<dev<<"\""<<std::endl;
                motor = new ::common::actuators::models::ActuatorTechnoSoft(dev.c_str(),atoi(slaveid.c_str()));
                if(motor==NULL){
                      throw chaos::CException(1, "Cannot allocate resources for C_TechnosoftMotor", "C_TechnosoftMotor::driverInit");
                }
            
            } else {
                 throw chaos::CException(1, "Bad parameters for C_TechnosoftMotor <serial port>,<slaveid>,<maxcurr:maxvoltage>", "OcemDD::driverInit");

            }
        } else {
            throw chaos::CException(1, "Unsupported driver", "C_TechnosoftMotor::driverInit");
        
        }
    } else {
            throw chaos::CException(1, "Malformed initialisation string", "C_TechnosoftMotor::driverInit");
    
    }
    
    std::string ver;
    motor->getSWVersion(ver,0);
    PSLAPP<<"Initialising PowerSupply Driver \""<<ver<<"\""<<std::endl;
    
}

