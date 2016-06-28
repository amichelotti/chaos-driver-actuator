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


#define SCCUAPP INFO_LOG(SCActuatorControlUnit)
#define SCCUDBG DBG_LOG(SCActuatorControlUnit)
#define SCCUERR ERR_LOG(SCActuatorControlUnit)

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


bool ::driver::actuator::SCActuatorControlUnit::setSpeed(const std::string &name,double value,size_t size){
        int err= -1;
          const double *speed = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");
          if(value>=0){
                    SCCUAPP << "set speed:"<<value<< "::" << speed;

                  err = actuator_drv->setSpeed(value);
          }
         return (err==chaos::ErrorCode::EC_NO_ERROR);
}
bool ::driver::actuator::SCActuatorControlUnit::setAcceleration(const std::string &name,double value,size_t size){
          int err= -1;
          const double *acc = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "acceleration");
	  SCCUAPP << "HANDLER setAcceleration" ;
          if(value>=0){
                  SCCUAPP << "set acceleration:"<<value<< "::" << acc;
                  err = actuator_drv->setAcceleration(value);
          }
         return (err==chaos::ErrorCode::EC_NO_ERROR);
}
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
  SCCUAPP <<"ALEDEBUG ALEDEBUG REQUESTING ACCESSOR AND DRIVER ON PREINIT " ;
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
            SCCUAPP << "WARNING attribute type  is unknown " ;
            continue; 
	}

	std::string attrName=json_attribute_name.asString();
	std::string attrDesc=json_attribute_description.asString();
	std::string datatype=json_attribute_type.asString();
	std::string datadirection=json_attribute_dir.asString();
	SCCUAPP << attrName <<" " <<  attrDesc <<" " << datatype << "("<<dtt<<")" << " " << datadirection;
	
	addAttributeToDataSet(attrName,attrDesc,dtt,DataType::Input);
	SCCUAPP << (*it)["name"] ;
    }

  }



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
                        DataType::Output);
  
  addAttributeToDataSet("position_sp",
                        "position Set Point",
                        DataType::TYPE_DOUBLE,
                        DataType::Output);
  
  addAttributeToDataSet("alarms",
                        "Alarms",
                        DataType::TYPE_INT64,
                        DataType::Output);

  addAttributeToDataSet("status_id",
                        "status_id",
                        DataType::TYPE_INT32,
                        DataType::Output);

  addAttributeToDataSet("status",
                        "status",
                        DataType::TYPE_STRING,
                        DataType::Output, 256);

addAttributeToDataSet("InitString",
                        "InitString",
                        DataType::TYPE_STRING,
                        DataType::Input, 256);
  
  
  addAttributeToDataSet("driver_timeout",
                        "Driver timeout in milliseconds",
                        DataType::TYPE_INT32,
                        DataType::Input);
  
 addAttributeToDataSet("command_timeout",
                        "Command timeout in milliseconds",
                        DataType::TYPE_INT32,
                        DataType::Input);
 
  addAttributeToDataSet("delta_setpoint",
                        "Delta of the setpoint",
                        DataType::TYPE_INT32,
                        DataType::Input);

  addAttributeToDataSet("setpoint_affinity",
                        "Delta of the setpoint",
                        DataType::TYPE_INT32,
                        DataType::Input);

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


  SCCUAPP <<"ALEDEBUG ALEDEBUG REQUESTING ACCESSOR  AND DRIVER   AFTER INIT " ;
  chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = getAccessoInstanceByIndex(0);
  if (actuator_accessor == NULL) {
    throw chaos::CException(-1, "Cannot retrieve the requested driver", __FUNCTION__);
  }
  actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor);
  if (actuator_drv == NULL) {
    throw chaos::CException(-2, "Cannot allocate driver resources", __FUNCTION__);
  }
  std::string *initString;//=(std::string*)getAttributeCache()->getROPtr<std::string>(DOMAIN_INPUT, "InitString") ;
  //if (initString->c_str() == NULL)
  {
	initString=new std::string();
	initString->assign("/dev/ttyr1c,myslit,/home/chaos/chaos-distrib/etc/common_actuators_technosoft/1setup001.t.zip,14,14");
	DPRINT("assigned by default");
  }

  

    // perfomed in driver initialization
  if (actuator_drv->init((void*)initString->c_str()) != 0) {
    throw chaos::CException(-3, "Cannot initialize actuator " + control_unit_instance, __FUNCTION__);
  }
  double *MDSSpeed=(double*)getAttributeCache()->getROPtr<std::string>(DOMAIN_INPUT, "speed") ;
  SCCUAPP <<"ALEDEBUG ALEDEBUG AFTER INIT READ MDS SPEED "<< *MDSSpeed ;
  
  //addAttributeToDataSet("maxSpeed", "max Speed in mm/s", DataType::TYPE_DOUBLE, DataType::Input);

  // performing power on
  if (actuator_drv->poweron(1) != 0) {
    throw chaos::CException(-3, "Cannot poweron actuator " + control_unit_instance, __FUNCTION__);
  }
  


  if (actuator_drv->getState(&state_id, state_str) != 0) {
    throw chaos::CException(-6, "Error getting the state of the actuator, possibily off", __FUNCTION__);
  }
  
  *status_id = state_id;
  //notify change on status_id cached attribute
  getAttributeCache()->setOutputDomainAsChanged();

  if (actuator_drv->getHWVersion(device_hw) == 0) {
    SCCUAPP << "hardware found: \"" << device_hw << "\"";
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
    actuator_drv->stopMotion();
    SCCUAPP << "Power off ";
    actuator_drv->poweron(0);
    SCCUAPP << "deinitializing ";
    actuator_drv->deinit();
    
}

//! restore the control unit to snapshot
#define RESTORE_LAPP SCCUAPP << "[RESTORE-" <<getCUID() << "] "
#define RESTORE_LERR SCCUERR << "[RESTORE-" <<getCUID() << "] "

bool ::driver::actuator::SCActuatorControlUnit::unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache *const snapshot_cache) throw(chaos::CException) {
//  RESTORE_LAPP << "Check if restore cache has the needed data";
/* 
 //check if in the restore cache we have all information we need
  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("status_id")) return false;
  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("polarity")) return false;
  if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("current_sp")) return false;

  RESTORE_LAPP << "Start the restore of the powersupply";
  uint64_t start_restore_time = chaos::common::utility::TimingUtil::getTimeStamp();
  try {
    bool cmd_result = true;
    //get actual state
    double *now_current_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "current_sp");
    int32_t *now_status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");
    int32_t *now_polarity = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "polarity");

    //chec the restore polarity
    int32_t restore_polarity = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "polarity")->getValuePtr<int32_t>();
    double restore_current_sp = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "current_sp")->getValuePtr<double>();
    //handle bipolar
    int is_bipolar=powersupply_drv->getFeatures()& ::common::powersupply::POWER_SUPPLY_FEAT_BIPOLAR;
   
    ///

    if ((*now_polarity != restore_polarity)&&(is_bipolar==0)) {
      //we need to change the polarity
      RESTORE_LAPP << "Change the polarity from:" << *now_polarity << " to:" << restore_polarity;

      //put in standby
      RESTORE_LAPP << "Put powersupply at setpoint 0";
      if (setCurrent(0.0)) {
        usleep(100000);
        RESTORE_LAPP << "Start the restore of the powersupply";
        if (powerStandby()) {
          usleep(100000);
          RESTORE_LAPP << "Powersupply is gone in standby";
        } else {
          LOG_AND_TROW(RESTORE_LERR, 2, "Power supply is not gone in standby");
        }
      } else {
        LOG_AND_TROW(RESTORE_LERR, 1, "Power supply is not gone to 0 ampere");
      }

      //set the polarity
      RESTORE_LAPP << "Apply new polarity";
      if (!setPolarity(restore_polarity)) {
        LOG_AND_TROW_FORMATTED(RESTORE_LERR, 3, "Power supply is not gone to restore polarity %1%", %restore_polarity);
      }
      usleep(100000);
    }

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


    usleep(100000);
    RESTORE_LAPP << "Apply new setpoint " << restore_current_sp;
    if (!setCurrent(restore_current_sp)) {
      LOG_AND_TROW_FORMATTED(RESTORE_LERR,
                             6,
                             "Power supply is not gone to restore 'current setpoint %1%' state",
                             %restore_current_sp);
    }
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LAPP << "[metric] Restore successfully achieved in " << restore_duration_in_ms << " milliseconds";
  } catch (CException &ex) {
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LAPP << "[metric] Restore has fault in " << restore_duration_in_ms << " milliseconds";
    throw ex;
  }
*/
  return false;

}

//-----------utility methdo for the restore operation---------
/*
bool ::driver::actuator::SCActuatorControlUnit::powerON(bool sync) {
  uint64_t cmd_id;
  bool result = true;
  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addInt32Value(CMD_PS_MODE_TYPE, 1);
  //send command
  submitBatchCommand(CMD_PS_MODE_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_AND_Stack);
  if (sync) {
    //! whait for the current command id to finisch
    result = waitOnCommandID(cmd_id);
  }
  return result;
}
*/



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



