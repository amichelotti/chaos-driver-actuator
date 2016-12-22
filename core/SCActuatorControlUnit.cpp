/*
 *	SCActuatorControlUnit
 *	!CHAOS
 *	Created by Alessandro D'Uffizi.
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

#include "SCActuatorControlUnit.h"
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <common/debug/core/debug.h>
#include <json/json.h>

//---commands----

#include "CmdACTDefault.h"
#include "CmdACTMoveRelative.h"
#include "CmdACTMoveAbsolute.h"
#include "CmdACTStopMotion.h"
#include "CmdACTHoming.h"
#include "CmdACTPoweron.h"
#include "CmdACTresetAlarms.h"
#include "CmdACTSetParameter.h"


using namespace chaos;

using namespace chaos::common::data;
using namespace chaos::common::batch_command;

using namespace chaos::cu::control_manager::slow_command;
using namespace chaos::cu::driver_manager::driver;


#define SCCUAPP INFO_LOG(SCActuatorControlUnit) << "[" << getDeviceID() << "] "
#define SCCUDBG DBG_LOG(SCActuatorControlUnit) << "[" << getDeviceID() << "] "
#define SCCUERR ERR_LOG(SCActuatorControlUnit) << "[" << getDeviceID() << "] "

PUBLISHABLE_CONTROL_UNIT_IMPLEMENTATION(::driver::actuator::SCActuatorControlUnit)

/*
 Construct a new CU with an identifier
 */
::driver::actuator::SCActuatorControlUnit::SCActuatorControlUnit(const string &_control_unit_id,
                                                                          const string &_control_unit_param,
                                                                          const ControlUnitDriverList &_control_unit_drivers)
    :
//call base constructor
    chaos::cu::control_manager::SCAbstractControlUnit(_control_unit_id,
                                                      _control_unit_param,
                                                      _control_unit_drivers) {
  actuator_drv = NULL;
}

/*
 Base destructor
 */
::driver::actuator::SCActuatorControlUnit::~SCActuatorControlUnit() {
  if (actuator_drv) {
    delete (actuator_drv);
  }
}
int ::driver::actuator::SCActuatorControlUnit::decodeType(const std::string& str_type, DataType::DataType& attribute_type) {
    int err = 0;
    if(str_type.compare("int32")==0) {
        attribute_type = DataType::TYPE_INT32;
    } else if(str_type.compare("uint32")==0) {
        attribute_type = DataType::TYPE_INT64;
    } else if(str_type.compare("int64")==0) {
        attribute_type = DataType::TYPE_INT64;
    } else if(str_type.compare("uint64")==0) {
        attribute_type = DataType::TYPE_INT64;
    } else if(str_type.compare("double")==0) {
        attribute_type = DataType::TYPE_DOUBLE;
    } else if(str_type.compare("string")==0) {
        attribute_type = DataType::TYPE_STRING;
    } else if(str_type.compare("binary")==0) {
        attribute_type = DataType::TYPE_BYTEARRAY;
    } else if(str_type.compare("boolean")==0 ) {
        attribute_type = DataType::TYPE_BOOLEAN;
    } else {
        err = -1;
    }

    return err;
}

/*
bool ::driver::actuator::SCActuatorControlUnit::setMovement(const std::string &name,int32_t value,size_t size){
          int err= -1;
          const int32_t *mov = getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "movement");
	  SCCUAPP << "HANDLER setMovement" ;
          if(value>=0){
                  SCCUAPP << "set movement:"<<value<< "::" << mov;
                  err = actuator_drv->setMovement(value);
          }
         return (err==chaos::ErrorCode::EC_NO_ERROR);
}
*/
/*
 Return the default configuration
 */
void ::driver::actuator::SCActuatorControlUnit::unitDefineActionAndDataset() throw(chaos::CException) {
  //install all command

  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTDefault), true);

  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTMoveRelative));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTMoveAbsolute));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTStopMotion));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTHoming));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTPoweron));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTresetAlarms));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTSetParameter));
  //setup the dataset
  
  chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = getAccessoInstanceByIndex(0);
  if (actuator_accessor == NULL) {
    throw chaos::CException(-1, "Cannot retrieve the requested driver", __FUNCTION__);
  }


  actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
  if (actuator_drv == NULL) {
    throw chaos::CException(-2, "Cannot allocate driver resources", __FUNCTION__);
  }
  std::string dataset;
  actuator_drv->sendDataset(dataset) ;
  SCCUAPP << "ALEDEBUG getting dataset from driver " << dataset;
  Json::Value                                 json_parameter;
  Json::Reader                                json_reader;


  //parse json string
  if(!json_reader.parse(dataset, json_parameter)) {
  SCCUAPP << "Bad Json parameter " << json_parameter;
  }
  else {
    const Json::Value& dataset_description = json_parameter["attributes"];
    for (Json::ValueConstIterator it = dataset_description.begin(); it != dataset_description.end();it++)
    {
 	const Json::Value& json_attribute_name = (*it)["name"];

        if(json_attribute_name.isNull()) {
            SCCUAPP << "WARNING attribute name is null " ;
            continue; 
        }
        if(!json_attribute_name.isString()) {
            SCCUAPP << "WARNING attribute name is not a string " ;
            continue; 
        }
 	const Json::Value& json_attribute_description = (*it)["description"];
        if(json_attribute_description.isNull()) {
            SCCUAPP << "WARNING attribute description is null " ;
            continue; 
        }
        if(!json_attribute_description.isString()) {
            SCCUAPP << "WARNING attribute description is not a string " ;
            continue; 
	}
 	const Json::Value& json_attribute_type = (*it)["datatype"];
        if(json_attribute_type.isNull()) {
            SCCUAPP << "WARNING attribute datatype is null " ;
            continue; 
        }
        if(!json_attribute_type.isString()) {
            SCCUAPP << "WARNING attribute datatype is not a string " ;
            continue; 
	}
 	const Json::Value& json_attribute_dir = (*it)["direction"];
        if(json_attribute_dir.isNull()) {
            SCCUAPP << "WARNING attribute direction is null " ;
            continue; 
        }
        if(!json_attribute_dir.isString()) {
            SCCUAPP << "WARNING attribute direction is not a string " ;
            continue; 
	}
        DataType::DataType dtt;
	if (decodeType(json_attribute_type.asString(),dtt) == -1) {
            SCCUAPP << "WARNING attribute type " <<json_attribute_type.asString()<< "  is unknown " ;
            continue; 
	}

	std::string attrName=json_attribute_name.asString();
	std::string attrDesc=json_attribute_description.asString();
	std::string datatype=json_attribute_type.asString();
	std::string datadirection=json_attribute_dir.asString();
	//SCCUAPP << attrName <<" " <<  attrDesc <<" " << datatype << "("<<dtt<<")" << " " << datadirection;
	
	addAttributeToDataSet(attrName,attrDesc,dtt,DataType::Input);
	//SCCUAPP << (*it)["name"] ;
    }

  }

   addAttributeToDataSet("axisID",
                        "axis ID for the motor",
                        DataType::TYPE_INT32,
                        DataType::Input);

   addAttributeToDataSet("readingType",
                        "readingType",
                        DataType::TYPE_INT32,
                        DataType::Input);
 

   addAttributeToDataSet("dev_state",
                        "Bit field device state",
                        DataType::TYPE_INT64,
                        DataType::Output);

          

  addAttributeToDataSet("position",
                        "position",
                        DataType::TYPE_DOUBLE,
                        DataType::Bidirectional);
  
  addAttributeToDataSet("alarms",
                        "Alarms",
                        DataType::TYPE_INT64,
                        DataType::Output);

  addAttributeToDataSet("alarmStr",
                        "alarm description string",
                        DataType::TYPE_STRING,
                        DataType::Output, 256);

  addAttributeToDataSet("status_id",
                        "status_id",
                        DataType::TYPE_INT32,
                        DataType::Output);

  addAttributeToDataSet("status",
                        "status",
                        DataType::TYPE_STRING,
                        DataType::Output, 256);

addAttributeToDataSet("ConfigString",
                        "ConfigString",
                        DataType::TYPE_STRING,
                        DataType::Input, 256);

addAttributeToDataSet("auxiliaryConfigParameters",
                        "parameter:value;parameter:value;",
                        DataType::TYPE_STRING,
                        DataType::Input, 512);
  
  addAttributeToDataSet("driver_timeout",
                        "Driver timeout in milliseconds",
                        DataType::TYPE_INT32,
                        DataType::Input);
  
 addAttributeToDataSet("setTimeout",
                        "Command timeout in milliseconds",
                        DataType::TYPE_INT32,
                        DataType::Input);
 
  addAttributeToDataSet("__delta_setpoint",
                        "Delta of the setpoint",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);

  addAttributeToDataSet("__setpoint_affinity",
                        "Delta of the setpoint",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);
  
  addAttributeToDataSet("__setpoint_affinity",
                        "Delta of the setpoint",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);

  
  addAttributeToDataSet("resolution",
                        "resolution on position",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);
  
  addAttributeToDataSet("positionWarningTHR_Timeout",
                        "Tolerance time for Threshold warning on Position",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);
  
addAttributeToDataSet("positionWarningTHR",
                        "Position warning threshold",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);
  
   addAttributeToDataSet("bypass",
                          "exclude HW changes",
                          DataType::TYPE_BOOLEAN,
                          DataType::Input);
   
   addAttributeToDataSet("stby",
                          "stdby management",
                          DataType::TYPE_BOOLEAN,
                          DataType::Output);
   
   addAttributeToDataSet("stopHoming",
                          "homing to be stopped flag",
                          DataType::TYPE_BOOLEAN,
                          DataType::Output);
 
/***************************ALARMS******************************************/
addAlarm("position_out_of_set",
            "Notify when a position has drifted away from set");


addAlarm("homing_operation_failed",
            "Notify when a homing operation has failed");

addAlarm("position_value_not_reached",
            "Notify when a moving operation has failed to reach the final set point ");

addAlarm("command_error",
            "Notify when a command action fails");
/***************************ALARMS******************************************/
addAlarm("DRIVER_COMMAND_ERROR",
            "Notify when a driver returns a generic command error");
addAlarm("DRIVER_COMMUNICATION_ERROR",
            "Notify when a comunication error has raised from the driver");
addAlarm("DRIVER_SHORT_CIRCUIT_PROTECTION",
            "Notify when the driver is in protection for short-circuit");
addAlarm("DRIVER_INVALID_SETUP_DATA",
            "Notify when the driver received a bad data setup");
addAlarm("DRIVER_CONTROL_ERROR",
            "Notify when the driver sends a generic  control error");
addAlarm("DRIVER_HALL_SENSOR_MISSING",
            "Notify when the driver sends a hall sensor missing alarm");
addAlarm("DRIVER_OVER_CURRENT",
            "Notify when the driver has an over current alarm");
addAlarm("DRIVER_I2T_ALARM",
            "Notify when the driver raise a I2T  alarm");
addAlarm("MOTOR_OVER_TEMPERATURE",
            "Notify when the motor is in over temperature");
addAlarm("DRIVE_OVER_TEMPERATURE",
            "Notify when the drive is in over temperature");
addAlarm("OVER_VOLTAGE_ALARM",
            "Notify when there is an over voltage on the system");
addAlarm("UNDER_VOLTAGE_ALARM",
            "Notify when there is an under voltage on the system");
addAlarm("READING_ALARM_PROBLEM",
            "Notify when the driver cannot execute the reading of alarms");
}

void ::driver::actuator::SCActuatorControlUnit::unitDefineCustomAttribute() {
  //addAttributeToDataSet("maxSpeedCustom", "max Speed in mm/s", DataType::TYPE_DOUBLE, DataType::Input);

}

// Abstract method for the initialization of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitInit() throw(CException) {
  SCCUAPP << "Starting unitInit";

  int err = 0;
  int state_id;
  std::string max_range;
  std::string min_range;
  std::string state_str;
  RangeValueInfo attr_info;

   int32_t *status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");
   SCCUAPP << "unitInit() dopo status_id";
   
   
   double *o_positionSP = (double*)getAttributeCache()->getRWPtr<double>(DOMAIN_INPUT, "position"); 
   SCCUAPP << "unitInit() dopo o_positionSP";
   
   double *o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");

  SCCUAPP <<"ALEDEBUG ALEDEBUG REQUESTING ACCESSOR  AND DRIVER   AFTER INIT " ;
    const bool* s_bypass=getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "bypass");

      chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = *s_bypass&&(getAccessoInstanceByIndex(1))?getAccessoInstanceByIndex(1):getAccessoInstanceByIndex(0);

  //chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = getAccessoInstanceByIndex(0);
  if (actuator_accessor == NULL) {
    throw chaos::CException(-1, "Cannot retrieve the requested driver", __FUNCTION__);
  }
  actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
  if (actuator_drv == NULL) {
    throw chaos::CException(-2, "Cannot allocate driver resources", __FUNCTION__);
  }
  std::string *initString =new std::string();
  char* ptStr=NULL, *auxStr=NULL;
  axID=getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
  ptStr=(char*)getAttributeCache()->getROPtr<char*>(DOMAIN_INPUT, "ConfigString");
  auxStr=(char*)getAttributeCache()->getROPtr<char*>(DOMAIN_INPUT, "auxiliaryConfigParameters");
  initString->assign(ptStr);
  if (initString->c_str() == NULL)
  {
	initString->assign("14,/home/chaos/chaos-distrib/etc/common_actuators_technosoft/1setup001.t.zip");
	DPRINT("assigned by default");
  }
  

    DPRINT("configuring driver from Control Unit ");
    DPRINT("config string is %s",initString->c_str() );
    // perfomed in driver initialization
  if (actuator_drv->configAxis((void*)initString->c_str()) != 0) {
    throw chaos::CException(-3, "Cannot initialize actuator " + control_unit_instance, __FUNCTION__);
  }
  // performing power on
    DPRINT("power on to Control Unit");
  if (actuator_drv->poweron(*axID,1) != 0) {
    throw chaos::CException(-3, "Cannot poweron actuator " + control_unit_instance, __FUNCTION__);
  }
  //parsing di auxiliary
    {
    char* param=NULL;
    char* value;
    param=strtok(auxStr,":");
    while (param)
    {
        value=NULL;
        value=strtok(NULL,";");
        if (value)
        {
            std::string PAR(param);
            std::string VAL(value);
            int ret;
            ret=actuator_drv->setParameter(*axID,PAR,VAL);
            if (ret)
                SCCUERR << "bad configuration for parameter " <<param << " value "<<value;
            else
                SCCUAPP << "Parameter "<<param<< "set to "<< value;
        }
        else break;
        param=strtok(NULL,":");
    }
    }

  if (actuator_drv->getState(*axID,&state_id, state_str) != 0) {
    throw chaos::CException(-6, "Error getting the state of the actuator, possibily off", __FUNCTION__);
  }
  
  *status_id = state_id;
  //notify change on status_id cached attribute
  getAttributeCache()->setOutputDomainAsChanged();

  if (actuator_drv->getHWVersion(*axID,device_hw) == 0) {
    SCCUAPP << "hardware found: \"" << device_hw << "\"";
  }
  ::common::actuators::AbstractActuator::readingTypes readTyp;
  double tmp_float = 0.0F;
  const int32_t *tmpInt =  getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
  readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
   if((err = actuator_drv->getPosition(*axID,readTyp,&tmp_float))==0){
		*o_position = tmp_float;
                *o_positionSP=tmp_float;
    } else {
		throw chaos::CException(-9, "Error getting initial position of the actuator", __FUNCTION__);
	}
  

}

// Abstract method for the start of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitStart() throw(CException) {
    
    
   
}

// Abstract method for the stop of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitStop() throw(CException) {

}

// Abstract method for the deinit of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitDeinit() throw(CException) {
    SCCUAPP << "Stop Motion ";
    actuator_drv->stopMotion(*axID);
    SCCUAPP << "Power off ";
    actuator_drv->poweron(*axID,0);
    SCCUAPP << "deinitializing ";
    actuator_drv->deinit(*axID);
    
}

//! restore the control unit to snapshot
#define RESTORE_LAPP SCCUAPP << "[RESTORE-" <<getCUID() << "] "
#define RESTORE_LERR SCCUERR << "[RESTORE-" <<getCUID() << "] "

bool ::driver::actuator::SCActuatorControlUnit::unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache *const snapshot_cache) throw(chaos::CException) {
  RESTORE_LAPP << "Check if restore cache has the needed data";
 //check if in the restore cache we have all information we need
  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("status_id")) return false;
  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("position")) return false;

  RESTORE_LAPP << "Start the restore of the powersupply";
  uint64_t start_restore_time = chaos::common::utility::TimingUtil::getTimeStamp();
 
  try {
    bool cmd_result = true;
    //get actual state
    double *now_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
    int32_t *now_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");

    double restore_position_sp = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "position")->getValuePtr<double>();

/*
    int32_t restore_status_id = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "status_id")->getValuePtr<int32_t>();
    if (*now_status_id != restore_status_id) {
      RESTORE_LAPP << "Change the status from:" << *now_status_id << " to:" << restore_status_id;
      //we need to change the sate
      switch (restore_status_id) {
        case 0x2:
          RESTORE_LAPP << "Put powersupply in on state to restore his status";
          if (!powerON()) {
            LOG_AND_TROW(RESTORE_LERR, 4, "Power supply is not gone to restore 'power on' state");
          }
          break;
        case 0x8:
          //set the powersupply on stand-by
          SCCUAPP << "Put powersupply in standby state to restore his status";
          if (setCurrent(0.0)) {
            if (powerStandby()) {
              RESTORE_LAPP << "Powersupply is gone in standby";
            } else {
              LOG_AND_TROW(RESTORE_LERR, 5, "Power supply is not gone in standby");
            }
          } else {
            LOG_AND_TROW(RESTORE_LERR, 6, "Power supply is not gone to 0 ampere");
          }
          break;

        default:
          return false;
          break;
      }
    }
*/

    usleep(100000);
    RESTORE_LAPP << "Apply new setpoint " << restore_position_sp;
    if (!setPosition(restore_position_sp)) {
      LOG_AND_TROW_FORMATTED(RESTORE_LERR,
                             6,
                             "Actuator is not gone to restore 'position setpoint %1%' state",
                             %restore_position_sp);
    }
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LAPP << "[metric] Restore successfully achieved in " << restore_duration_in_ms << " milliseconds";
  } catch (CException &ex) {
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LAPP << "[metric] Restore has fault in " << restore_duration_in_ms << " milliseconds";
    throw ex;
  }

  return false;

}

//-----------utility methdo for the restore operation---------

bool ::driver::actuator::SCActuatorControlUnit::setPosition(bool sync) {
  uint64_t cmd_id;
  bool result = true;
  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addInt32Value(CMD_ACT_MM_OFFSET, 1);
  //send command
  submitBatchCommand(CMD_ACT_MOVE_ABSOLUTE_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_AND_STACK);
  if (sync) {
    result = waitOnCommandID(cmd_id);
  }
  return result;
}




bool ::driver::actuator::SCActuatorControlUnit::waitOnCommandID(uint64_t cmd_id) {
  std::auto_ptr<CommandState> cmd_state;
  do {
    cmd_state = getStateForCommandID(cmd_id);
    if (!cmd_state.get()) break;

    switch (cmd_state->last_event) {
      case BatchCommandEventType::EVT_QUEUED:
        SCCUAPP << cmd_id << " -> QUEUED";
        break;
      case BatchCommandEventType::EVT_RUNNING:
        SCCUAPP << cmd_id << " -> RUNNING";
        break;
      case BatchCommandEventType::EVT_WAITING:
        SCCUAPP << cmd_id << " -> WAITING";
        break;
      case BatchCommandEventType::EVT_PAUSED:
        SCCUAPP << cmd_id << " -> PAUSED";
        break;
      case BatchCommandEventType::EVT_KILLED:
        SCCUAPP << cmd_id << " -> KILLED";
        break;
      case BatchCommandEventType::EVT_COMPLETED:
        SCCUAPP << cmd_id << " -> COMPLETED";
        break;
      case BatchCommandEventType::EVT_FAULT:
        SCCUAPP << cmd_id << " -> FALUT";
        break;
    }
    //wait some times
    usleep(500000);
  } while (cmd_state->last_event != BatchCommandEventType::EVT_COMPLETED &&
      cmd_state->last_event != BatchCommandEventType::EVT_FAULT &&
      cmd_state->last_event != BatchCommandEventType::EVT_KILLED);
  return (cmd_state.get() &&
      cmd_state->last_event == BatchCommandEventType::EVT_COMPLETED);
}



