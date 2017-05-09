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
#include "CmdACTHardReset.h"


using namespace chaos;

using namespace chaos::common::data;
using namespace chaos::common::batch_command;

using namespace chaos::cu::control_manager::slow_command;
using namespace chaos::cu::driver_manager::driver;
using namespace chaos::cu::control_manager;


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
bool ::driver::actuator::SCActuatorControlUnit::setPower(const std::string &name,bool value,uint32_t size){
          int err= -1;
          int *axis;
	  uint64_t cmd_id;
	  SCCUAPP << "HANDLER set Power" ;
 	  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
          cmd_pack->addInt32Value(CMD_ACT_POWERON_VALUE, value);
    //send command
            submitBatchCommand(CMD_ACT_POWERON_ALIAS,
            cmd_pack.release(),
            cmd_id,
            0,
            50,
            SubmissionRuleType::SUBMIT_NORMAL);
         return (err==chaos::ErrorCode::EC_NO_ERROR);
}


bool ::driver::actuator::SCActuatorControlUnit::moveAt(const std::string &name,double value,uint32_t size){
          int err= -1;
          int *axis;
	  uint64_t cmd_id;

	  SCCUAPP << "HANDLER Move at" ;
 	  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
          cmd_pack->addDoubleValue(CMD_ACT_MM_OFFSET, value);

    //send command
    submitBatchCommand(CMD_ACT_MOVE_ABSOLUTE_ALIAS,
            cmd_pack.release(),
            cmd_id,
            0,
            50,
            SubmissionRuleType::SUBMIT_NORMAL);
          SCCUAPP << "move to:"<<value ;

         return true;
}

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
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTHardReset));
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

    addAttributeToDataSet("LastHomingTime",
                          "timestamp with the last homing",
                          DataType::TYPE_INT64,
                          DataType::Output);
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

     addAttributeToDataSet("powerOn",
                            "stdby management",
                            DataType::TYPE_BOOLEAN,
                            DataType::Bidirectional);

     addAttributeToDataSet("PositiveLimitSwitchActive",
                            "if on, positive limit switch are currently pressed",
                            DataType::TYPE_BOOLEAN,
                            DataType::Output);

     addAttributeToDataSet("NegativeLimitSwitchActive",
                            "if on, negative limit switch are currently pressed",
                            DataType::TYPE_BOOLEAN,
                            DataType::Output);

     addAttributeToDataSet("stopHoming",
                            "homing to be stopped flag",
                            DataType::TYPE_BOOLEAN,
                            DataType::Output);




  std::string dataset;
  actuator_drv->sendDataset(dataset) ;
  SCCUAPP << "DATASETVARIABLE getting dataset from driver " << dataset;
  Json::Value                                 json_parameter;
  Json::Reader                                json_reader;


  //parse json string
  if(!json_reader.parse(dataset, json_parameter)) {
  SCCUAPP << "Bad Json parameter " << json_parameter;
  }
  else
  {
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

   
 
 addHandlerOnInputAttributeName< ::driver::actuator::SCActuatorControlUnit, double >(this,
            &::driver::actuator::SCActuatorControlUnit::moveAt,
            "position");

 addHandlerOnInputAttributeName< ::driver::actuator::SCActuatorControlUnit, bool >(this,
            &::driver::actuator::SCActuatorControlUnit::setPower,
            "powerOn");
/***************************ALARMS******************************************/
addStateVariable(StateVariableTypeAlarmCU,"position_out_of_set",
            "Notify when a position has drifted away from set");

addStateVariable(StateVariableTypeAlarmCU,"position_value_not_reached",
            "Notify when a moving operation has failed to reach the final set point ");


addStateVariable(StateVariableTypeAlarmCU,"powerOn_out_of_set",
            "Notify when a poweOn state in out of set");

addStateVariable(StateVariableTypeAlarmCU,"powerOn_value_not_reached",
            "Notify when a powerOn has failed to reach the final set point ");


addStateVariable(StateVariableTypeAlarmCU,"homing_operation_failed",
            "Notify when a homing operation has failed");



addStateVariable(StateVariableTypeAlarmCU,"command_error",
            "Notify when a command action fails");
/***************************ALARMS******************************************/
addStateVariable(StateVariableTypeAlarmDEV,"EMERGENCY_LOCK_ENABLED",
            "Notify when the emergency lock is active");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_COMMAND_ERROR",
            "Notify when a driver returns a generic command error");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_COMMUNICATION_ERROR",
            "Notify when a comunication error has raised from the driver");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_SHORT_CIRCUIT_PROTECTION",
            "Notify when the driver is in protection for short-circuit");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_INVALID_SETUP_DATA",
            "Notify when the driver received a bad data setup");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_CONTROL_ERROR",
            "Notify when the driver sends a generic  control error");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_HALL_SENSOR_MISSING",
            "Notify when the driver sends a hall sensor missing alarm");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_OVER_CURRENT",
            "Notify when the driver has an over current alarm");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVER_I2T_ALARM",
            "Notify when the driver raise a I2T  alarm");
addStateVariable(StateVariableTypeAlarmDEV,"MOTOR_OVER_TEMPERATURE",
            "Notify when the motor is in over temperature");
addStateVariable(StateVariableTypeAlarmDEV,"DRIVE_OVER_TEMPERATURE",
            "Notify when the drive is in over temperature");
addStateVariable(StateVariableTypeAlarmDEV,"OVER_VOLTAGE_ALARM",
            "Notify when there is an over voltage on the system");
addStateVariable(StateVariableTypeAlarmDEV,"UNDER_VOLTAGE_ALARM",
            "Notify when there is an under voltage on the system");
addStateVariable(StateVariableTypeAlarmDEV,"READING_ALARM_PROBLEM",
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
   
   
   double *o_positionSP = (double*)getAttributeCache()->getRWPtr<double>(DOMAIN_INPUT, "position"); 
   uint64_t *homingDone = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "LastHomingTime");
   double *o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
   axID=getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
    const bool* s_bypass=getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "bypass");

      chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = *s_bypass&&(getAccessoInstanceByIndex(1))?getAccessoInstanceByIndex(1):getAccessoInstanceByIndex(0);

  //chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = getAccessoInstanceByIndex(0);
  if (actuator_accessor == NULL) {
    throw chaos::CFatalException(-1, "Cannot retrieve the requested driver", __FUNCTION__);
  }
  actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
  if (actuator_drv == NULL) {
    throw chaos::CFatalException(-2, "Cannot allocate driver resources", __FUNCTION__);
  }
  char* ptStr=NULL, *auxStr=NULL;

  ptStr=(char*)getAttributeCache()->getROPtr<char*>(DOMAIN_INPUT, "ConfigString");
  if(ptStr==NULL || *ptStr ==0){
	    throw chaos::CFatalException(-3, "You must provide a configuration string " + control_unit_instance, __FUNCTION__);

  }
  auxStr=(char*)getAttributeCache()->getROPtr<char*>(DOMAIN_INPUT, "auxiliaryConfigParameters");


  SCCUDBG<<"configuring driver from Control Unit ";
  SCCUDBG<<"config string is '"<<ptStr<<"'";
    // perfomed in driver initialization
  if ((err=actuator_drv->configAxis((void*)ptStr)) != 0) {
    throw chaos::CFatalException(err, "Cannot configure axis " + getDeviceID(), __FUNCTION__);
  }
  *homingDone=0;
  // performing power on
/*
  SCCUDBG<<"power on to Control Unit";
  if ((err=actuator_drv->poweron(*axID,1) != 0)) {
    throw chaos::CFatalException(err, "Cannot poweron actuator " + getDeviceID(), __FUNCTION__);
  }
*/
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

  if ((err=actuator_drv->getState(*axID,&state_id, state_str)) != 0) {
    throw chaos::CFatalException(err, "Error getting the state of the actuator", __FUNCTION__);
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
		throw chaos::CFatalException(err, "Error getting initial position of the actuator", __FUNCTION__);
	}
  
   metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("Initialization of axis '%1% done configuration '%2%' ") %*axID % ptStr) );

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
  RESTORE_LAPP << "Restore Check if  cache for position";
  if (!snapshot_cache->getSharedDomain(DOMAIN_INPUT).hasAttribute("position")) return false;

  RESTORE_LAPP << "Start the restore of the actuator";
  uint64_t start_restore_time = chaos::common::utility::TimingUtil::getTimeStamp();
  if (snapshot_cache == NULL) {
	RESTORE_LERR << "cache nulla" ;
	return false;
  }

  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("powerOn"))
  {
	  RESTORE_LERR << " missing powerOn to restore" ;
	  if (!snapshot_cache->getSharedDomain(DOMAIN_INPUT).hasAttribute("speed"))
	  return false;

  }


    double restore_position_sp = *snapshot_cache->getAttributeValue(DOMAIN_INPUT, "position")->getValuePtr<double>();
  RESTORE_LAPP << "Restore Trying to set position at " << restore_position_sp;
  bool restore_power_sp = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "powerOn")->getValuePtr<bool>();
  RESTORE_LAPP << "Restore Trying to set power at " << restore_power_sp;



  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,CHAOS_FORMAT("start restore \"%1%\" (axis %2%) to position %3% ",%getDeviceID() %*axID %restore_position_sp));
 
  try {
    bool cmd_result = true;
    //get actual state
    double *now_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
    int32_t *now_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");

    //setBusyFlag(true,1);
    setBusyFlag(true);
    if (!setPowerOn(true)) {
    	  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Error applying power on during restore \"%1%\" (axis %2%) to position %3% ",%getDeviceID() %*axID %restore_position_sp));
	  //setBusyFlag(false,-1);
	  setBusyFlag(false);
 	  return false;
    } 
    sleep(1);

    if (!setPosition(restore_position_sp)) {
    	  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Error restoring \"%1%\" (axis %2%) to position %3% ",%getDeviceID() %*axID %restore_position_sp));

	  setBusyFlag(false);
	  //setBusyFlag(false,-1);
    	  return false;
    }
    sleep(1);
    if (restore_power_sp == false) 
    {
	if (!setPowerOn(restore_power_sp)) 
	{
    	  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Error restoring power on during restore \"%1%\" (axis %2%) to value %3% ",%getDeviceID() %*axID %restore_power_sp));
	  setBusyFlag(false);
 	  return false;
	  
	}
    }
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
	metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,CHAOS_FORMAT("Restored \"%1%\" (axis %2%) to position %3% in %4%",%getDeviceID() %*axID %restore_position_sp %restore_duration_in_ms));
    setBusyFlag(false);

    //setBusyFlag(false,-1);
    setBusyFlag(false);
    return true;
  } catch (CException &ex) {
    //setBusyFlag(false,-1);
    setBusyFlag(false);
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LAPP << "[metric] Restore has fault in " << restore_duration_in_ms << " milliseconds";
    throw ex;
  }

  return false;

}

//-----------utility methdo for the restore operation---------

bool ::driver::actuator::SCActuatorControlUnit::setPosition(double val,bool sync) {
  uint64_t cmd_id;
  bool result = true;
  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addDoubleValue(CMD_ACT_MM_OFFSET, val);
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

bool  ::driver::actuator::SCActuatorControlUnit::setPowerOn(bool value,bool sync) {
	uint64_t cmd_id;
	bool result = true;
  	std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  	cmd_pack->addInt32Value(CMD_ACT_POWERON_VALUE, value);
  	//send command
  	submitBatchCommand(CMD_ACT_POWERON_ALIAS,
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



