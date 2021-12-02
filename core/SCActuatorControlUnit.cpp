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
#include <common/misc/driver/ConfigDriverMacro.h>
#include <chaos/common/data/CDataWrapper.h>
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
#include "CmdACTSoftHoming.h"

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
    : hasPoi(false),//call base constructor
      chaos::cu::control_manager::SCAbstractControlUnit(_control_unit_id,
                                                        _control_unit_param,
                                                        _control_unit_drivers)
{

  actuator_drv = NULL;
 	chaos::common::data::CDataWrapper p;
	if(getCUParam(p)==0){
    hasPoi=p.hasKey(CMD_ACT_MOVE_POI)&&(p.getAllKey().size()>0); 
    if(hasPoi){
      SCCUDBG<<"POI:"<<p.getCompliantJSONString();
    }
  }	
}

/*
 Base destructor
 */
::driver::actuator::SCActuatorControlUnit::~SCActuatorControlUnit()
{

  if (actuator_drv)
  {
    delete (actuator_drv);
  }
}
int ::driver::actuator::SCActuatorControlUnit::decodeType(const std::string &str_type, DataType::DataType &attribute_type)
{
  int err = 0;
  if (str_type.compare("int32") == 0)
  {
    attribute_type = DataType::TYPE_INT32;
  }
  else if (str_type.compare("uint32") == 0)
  {
    attribute_type = DataType::TYPE_INT64;
  }
  else if (str_type.compare("int64") == 0)
  {
    attribute_type = DataType::TYPE_INT64;
  }
  else if (str_type.compare("uint64") == 0)
  {
    attribute_type = DataType::TYPE_INT64;
  }
  else if (str_type.compare("double") == 0)
  {
    attribute_type = DataType::TYPE_DOUBLE;
  }
  else if (str_type.compare("string") == 0)
  {
    attribute_type = DataType::TYPE_STRING;
  }
  else if (str_type.compare("binary") == 0)
  {
    attribute_type = DataType::TYPE_BYTEARRAY;
  }
  else if (str_type.compare("boolean") == 0)
  {
    attribute_type = DataType::TYPE_BOOLEAN;
  }
  else
  {
    err = -1;
  }

  return err;
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, const chaos::common::data::CDataVariant&  value){
  int ret;
  SCCUDBG << "Variant PROP:" << name << " VALUE:" << value.asString();
  ret = actuator_drv->setParameter(*axID, name, value.asString());
  this->updateAuxiliaryParameters();
  return true;
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, const chaos::common::data::CDataWrapper&  value){
    SCCUDBG << "CDW PROP:" << name << " VALUE:" << value.getJSONString();

  return true;
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, int32_t value, uint32_t size)
{
  int ret;
  SCCUDBG << "SET IPROP:" << name << " VALUE:" << value;
  string valStr = ChaosToString(value);
  ret = actuator_drv->setParameter(*axID, (std::string)name, valStr);


  if ((name == "useIU") && (ret==0))
  {
	  int32_t* o_useUI = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "useSteps");
	  *o_useUI = atoi(valStr.c_str());
	  SCCUDBG <<" ret is" << ret << " setting new value for useSteps then updating auxiliary parameters";
	  getAttributeCache()->setOutputDomainAsChanged();
	  

	  
  }
  this->updateAuxiliaryParameters();
  return (ret == 0);
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, double value, uint32_t size)
{
  int ret;
  SCCUDBG << "SET IPROP:" << name << " VALUE:" << value;
  string valStr = ChaosToString(value);
  ret = actuator_drv->setParameter(*axID, (std::string)name, valStr);
  this->updateAuxiliaryParameters();
  return (ret == 0);
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, int64_t value, uint32_t size)
{
  int ret;
  SCCUDBG << "SET IPROP:" << name << " VALUE:" << value;
  string valStr = ChaosToString(value);
  ret = actuator_drv->setParameter(*axID, (std::string)name, valStr);
  this->updateAuxiliaryParameters();
  return (ret == 0);
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, bool value, uint32_t size)
{
  int ret;
  SCCUDBG << "SET IPROP:" << name << " VALUE:" << value;
  string valStr = ChaosToString(value);
  ret = actuator_drv->setParameter(*axID, (std::string)name, valStr);
  this->updateAuxiliaryParameters();
  return (ret == 0);
}
bool ::driver::actuator::SCActuatorControlUnit::setProp(const std::string &name, std::string value, uint32_t size)
{
  int ret;
  SCCUDBG << "SET IPROP:" << name << " VALUE:" << value;
  //string valStr=ChaosToString(value);
  ret = actuator_drv->setParameter(*axID, (std::string)name, value);
  this->updateAuxiliaryParameters();
  return (ret == 0);
}
bool ::driver::actuator::SCActuatorControlUnit::setPower(const std::string &name, bool value, uint32_t size)
{
  int err = -1;
  uint64_t cmd_id;
  int32_t vvv = 0;

  if (value == true)
  {
    vvv = 1;
  }
  else
  {
    vvv = 0;
  }
  SCCUDBG << "HANDLER set Power vslue = " << value << " and conversion is " << vvv;

  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addInt32Value(CMD_ACT_POWERON_VALUE, vvv);
  //send command
  submitBatchCommand(CMD_ACT_POWERON_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_NORMAL);
  return (err == chaos::ErrorCode::EC_NO_ERROR);
}

bool ::driver::actuator::SCActuatorControlUnit::moveAt(const std::string &name, double value, uint32_t size)
{
  uint64_t cmd_id;

  SCCUDBG << "HANDLER Move at " << value;
  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addDoubleValue(CMD_ACT_MM_OFFSET, value);

  //send command
  submitBatchCommand(CMD_ACT_MOVE_ABSOLUTE_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_NORMAL);
  SCCUDBG << "move to:" << value;

  return true;
}

bool ::driver::actuator::SCActuatorControlUnit::moveAt(const std::string &name, std::string value, uint32_t size)
{
  uint64_t cmd_id;

  SCCUDBG << "HANDLER Move at '" << value<<"'";
  std::auto_ptr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addStringValue(CMD_ACT_MOVE_POI, value);

  //send command
  submitBatchCommand(CMD_ACT_MOVE_ABSOLUTE_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_NORMAL);
  SCCUDBG << "move to:" << value;

  return true;
}

void ::driver::actuator::SCActuatorControlUnit::updateAuxiliaryParameters()
{
	int32_t* inSteps = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "useSteps");
	ChaosStringVector driverDefinedAttributes;
	auxiliarydataset.getAllKey(driverDefinedAttributes);
	
	for (ChaosStringVector::iterator it = driverDefinedAttributes.begin(); it != driverDefinedAttributes.end(); it++)
	{
		SCCUDBG << "Getting Parameter over key : " << (*it);
		CDWUniquePtr auxVar=auxiliarydataset.getCSDataValue(*it);
		
		std::string tmpStr;
		actuator_drv->getParameter(*axID, (*it), tmpStr);
		switch ((*auxVar).getInt32Value("cudk_ds_attr_type"))
		{
		case chaos::DataType::TYPE_DOUBLE:
		{
			double* tmpPointer = getAttributeCache()->getRWPtr<double>(DOMAIN_INPUT, (*it));
			*tmpPointer = (double)atof(tmpStr.c_str());
			break;
		}
		case chaos::DataType::TYPE_INT32:
		{
			int32_t* tmpPointer = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_INPUT, (*it));
			*tmpPointer = (int32_t)atoi(tmpStr.c_str());
			if ((*it) == "useIU")
			{
				*inSteps = atoi(tmpStr.c_str());
				getAttributeCache()->setOutputDomainAsChanged();
			}
			break;
		}
		case chaos::DataType::TYPE_INT64:
		{
			int64_t* tmpPointer = getAttributeCache()->getRWPtr<int64_t>(DOMAIN_INPUT, (*it));
			*tmpPointer = (int64_t)atol(tmpStr.c_str());
			break;
		}
		case chaos::DataType::TYPE_BOOLEAN:
		{
			bool* tmpPointer = getAttributeCache()->getRWPtr<bool>(DOMAIN_INPUT, (*it));
			*tmpPointer = (atoi(tmpStr.c_str()) == 0) ? false : true;
			break;
		}
		case chaos::DataType::TYPE_STRING:
		{
			char* tmpPointer = getAttributeCache()->getRWPtr<char>(DOMAIN_INPUT, (*it));
			//*tmpPointer=( char*)tmpStr.c_str();
			tmpPointer = (char*)tmpStr.c_str();
			break;
		}

		default:
			break;
		}
	}
	getAttributeCache()->setInputDomainAsChanged();

}

void ::driver::actuator::SCActuatorControlUnit::unitDefineCustomAttribute() {
    //here are defined the custom shared variable
    bool stop_homing=0;
    getAttributeCache()->addCustomAttribute("stopHoming", sizeof(bool), chaos::DataType::TYPE_BOOLEAN);
    getAttributeCache()->setCustomAttributeValue("stopHoming", &stop_homing, sizeof(bool));

}
/*
 Return the default configuration
 */
void ::driver::actuator::SCActuatorControlUnit::unitDefineActionAndDataset() 
{
  //install all command

  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTDefault), true);

  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTMoveRelative));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTMoveAbsolute));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTStopMotion));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTHoming));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTPoweron));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTresetAlarms));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTHardReset));
  installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTSoftHoming));
  //installCommand(BATCH_COMMAND_GET_DESCRIPTION(CmdACTSetParameter));
  //setup the dataset

  chaos::cu::driver_manager::driver::DriverAccessor *actuator_accessor = getAccessoInstanceByIndex(0);
  if (actuator_accessor == NULL)
  {
    throw chaos::CException(-1, "Cannot retrieve the requested driver", __FUNCTION__);
  }

  actuator_drv = new chaos::driver::actuator::ChaosActuatorInterface(actuator_accessor,getDeviceID());
  if (actuator_drv == NULL)
  {
    throw chaos::CException(-2, "Cannot allocate driver resources", __FUNCTION__);
  }

   addAttributeToDataSet("axisID",
                        "axis ID for the motor",
                        DataType::TYPE_INT32,
                        DataType::Input);

  addAttributeToDataSet("readingType",
                        "0:encoder,1:counter",
                        DataType::TYPE_INT32,
                        DataType::Input);

  addAttributeToDataSet("position",
                        "position",
                        DataType::TYPE_DOUBLE,
                        DataType::Bidirectional);
  /*addAttributeToDataSet("speed",
                        "Speed",
                        DataType::TYPE_DOUBLE,
                        DataType::Input);*/
                      
  addAttributeToDataSet("LastHomingTime",
                        "timestamp with the last homing",
                        DataType::TYPE_INT64,
                        DataType::Output);
  addAttributeToDataSet("KindOfHomingDone",
                        "0 no home, 1 real homing, 2 soft homing",
                        DataType::TYPE_INT32,
                        DataType::Output);
  addAttributeToDataSet("alarms",
                        "Device alarms BitMask",
                        DataType::TYPE_INT64,
                        DataType::Output);

  addAttributeToDataSet("alarmStr",
                        "alarm description string",
                        DataType::TYPE_STRING,
                        DataType::Output, 256);

  addAttributeToDataSet("status_id",
                        "Device Status BitMask ",
                        DataType::TYPE_INT32,
                        DataType::Output);

  addAttributeToDataSet("status",
                        "status",
                        DataType::TYPE_STRING,
                        DataType::Output, 256);

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

  
  addAttributeToDataSet("useSteps",
                        "if true motor will be controlled with steps as measure unit",
                        DataType::TYPE_INT32,
                        DataType::Output);

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

  addAttributeToDataSet("home",
                        "homing reached",
                        DataType::TYPE_BOOLEAN,
                        DataType::Output);

    addAttributeToDataSet("Lock",
                        "1: lock home, 2 lock all movements",
                        DataType::TYPE_INT32,
                        DataType::Input);
  addAttributeToDataSet("AvoidPositionRecoverOnInit",
                        "if true, do not place motor in soft_homing on Init",
                        DataType::TYPE_BOOLEAN,
                        DataType::Input);

if(hasPoi){
  addAttributeToDataSet("POI",
                        "Point of Interest",
                        DataType::TYPE_STRING,
                        DataType::Bidirectional,256);
  addHandlerOnInputAttributeName< ::driver::actuator::SCActuatorControlUnit, std::string>(this,
                                                                                    &::driver::actuator::SCActuatorControlUnit::moveAt,
                                                                                    "POI");                      

}
  std::string dataset;
  actuator_drv->listParameters(dataset);
  SCCUDBG << "DATASETVARIABLE getting dataset from driver :'"<< dataset<<"'";
  
  if(dataset.size()){
      CDataWrapper conf;
      // translate from user to chaos keys
      int         typ  = chaos::DataType::TYPE_UNDEFINED;
      conf.setSerializedJsonData(dataset.c_str());
      if(conf.hasKey("attributes")&&conf.isVectorValue("attributes")){
        chaos::common::data::CMultiTypeDataArrayWrapperSPtr  v=conf.getVectorValue("attributes");
        for(int cnt=0;cnt<v->size();cnt++){
          chaos::common::data::CDWUniquePtr val=v->getCDataWrapperElementAtIndex(cnt);
          std::string name=val->getStringValue("name");
          std::string desc;
          CDataWrapper norm;
          if (val->hasKey("description")) {  // TO REMOVE SOON
            desc = val->getStringValue("description");
           }
           const std::string str_type = boost::algorithm::to_lower_copy(val->getStringValue("datatype"));
          if (str_type.compare("int32") == 0) {
            typ = DataType::TYPE_INT32;
          } else if (str_type.compare("uint32") == 0) {
            typ = DataType::TYPE_INT64;
          } else if (str_type.compare("int64") == 0) {
            typ = DataType::TYPE_INT64;
          } else if (str_type.compare("uint64") == 0) {
            typ = DataType::TYPE_INT64;
          } else if (str_type.compare("double") == 0) {
            typ = DataType::TYPE_DOUBLE;
          } else if (str_type.compare("string") == 0) {
            typ = DataType::TYPE_STRING;
          } else if (str_type.compare("binary") == 0) {
            typ = DataType::TYPE_BYTEARRAY;
          } else if (str_type.compare("boolean") == 0) {
            typ = DataType::TYPE_BOOLEAN;
          } else {
            typ = DataType::TYPE_DOUBLE;
          }
          norm.addStringValue(chaos::ControlUnitNodeDefinitionKey::CONTROL_UNIT_DATASET_ATTRIBUTE_NAME,name);
          norm.addInt32Value(chaos::ControlUnitNodeDefinitionKey::CONTROL_UNIT_DATASET_ATTRIBUTE_TYPE,typ);
          norm.addStringValue(chaos::ControlUnitNodeDefinitionKey::CONTROL_UNIT_DATASET_ATTRIBUTE_DESCRIPTION,desc);
          //norm.addStringValue(chaos::ControlUnitNodeDefinitionKey::CONTROL_UNIT_DATASET_ATTRIBUTE_UNIT,desc);
          //CONTROL_UNIT_DATASET_ATTRIBUTE_DIRECTION
          auxiliarydataset.addCSDataValue(name,norm);


        }
          SCCUDBG << "Adding driver attributes :'"<< auxiliarydataset.getJSONString()<<"'";

          setDriverInfo(auxiliarydataset); // create into custom dataset an entry with the key: CONTROL_UNIT_DRIVER_INFO = cudk_driver_info

        }
 
      }
     
  


  addHandlerOnInputAttributeName< ::driver::actuator::SCActuatorControlUnit, double>(this,
                                                                                    &::driver::actuator::SCActuatorControlUnit::moveAt,
                                                                                    "position");

  addHandlerOnInputAttributeName< ::driver::actuator::SCActuatorControlUnit, bool>(this,
                                                                                  &::driver::actuator::SCActuatorControlUnit::setPower,
                                                                                  "powerOn");

   addInputAndHandlerOnEachKeyOf< ::driver::actuator::SCActuatorControlUnit>(this,                                                                             
                                                                                 &::driver::actuator::SCActuatorControlUnit::setProp,
                                                                                 auxiliarydataset);
  addHandlerOnCustomDriverAttributes< ::driver::actuator::SCActuatorControlUnit>(this,                                                                             
                                                                                 &::driver::actuator::SCActuatorControlUnit::setProp);                                                                               
  /***************************ALARMS******************************************/
 

 

  addStateVariable(StateVariableTypeAlarmCU, "powerOn_out_of_set",
                   "Notify when a poweOn state in out of set");

  addStateVariable(StateVariableTypeAlarmCU, "powerOn_value_not_reached",
                   "Notify when a powerOn has failed to reach the final set point ");

  addStateVariable(StateVariableTypeAlarmCU, "homing_operation_failed",
                   "Notify when a homing operation has failed");
  
   addStateVariable(StateVariableTypeAlarmCU, "action_prevented_by_lock_in_configuration",
                   "Notify when a user issues a movement operation when it is prevented via Lock Input parameter");

  addStateVariable(StateVariableTypeAlarmCU, "home_lost",
	  "Notify when the current home position is no more valid");

  addStateVariable(StateVariableTypeAlarmCU, "command_error",
                   "Notify when a command action fails");

/* supported in framework*/
  /*addStateVariable(StateVariableTypeAlarmCU, "user_command_failed",
	  "Notify when a batch command action fails");*/

  /***************************ALARMS******************************************/
  addStateVariable(StateVariableTypeAlarmDEV, "EMERGENCY_LOCK_ENABLED",
                   "Notify when the emergency lock is active");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_COMMAND_ERROR",
                   "Notify when a driver returns a generic command error");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_COMMUNICATION_ERROR",
                   "Notify when a comunication error has raised from the driver");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_SHORT_CIRCUIT_PROTECTION",
                   "Notify when the driver is in protection for short-circuit");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_INVALID_SETUP_DATA",
                   "Notify when the driver received a bad data setup");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_CONTROL_ERROR",
                   "Notify when the driver sends a generic  control error");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_HALL_SENSOR_MISSING",
                   "Notify when the driver sends a hall sensor missing alarm");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_OVER_CURRENT",
                   "Notify when the driver has an over current alarm");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVER_I2T_ALARM",
                   "Notify when the driver raise a I2T  alarm");
  addStateVariable(StateVariableTypeAlarmDEV, "MOTOR_OVER_TEMPERATURE",
                   "Notify when the motor is in over temperature");
  addStateVariable(StateVariableTypeAlarmDEV, "DRIVE_OVER_TEMPERATURE",
                   "Notify when the drive is in over temperature");
  addStateVariable(StateVariableTypeAlarmDEV, "OVER_VOLTAGE_ALARM",
                   "Notify when there is an over voltage on the system");
  addStateVariable(StateVariableTypeAlarmDEV, "UNDER_VOLTAGE_ALARM",
                   "Notify when there is an under voltage on the system");
  addStateVariable(StateVariableTypeAlarmDEV, "READING_ALARM_PROBLEM",
                   "Notify when the driver cannot execute the reading of alarms");
}


// Abstract method for the initialization of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitInit()
{
  SCCUDBG << "unitInit";
  std::string state_str;
  int err = -1;
  int state_id;
  
  int32_t *status_id = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "status_id");

  double *o_positionSP = (double *)getAttributeCache()->getRWPtr<double>(DOMAIN_INPUT, "position");
  uint64_t *homingDone = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "LastHomingTime");
  double *o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
  axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
  if(axID==NULL){
        throw chaos::CFatalException(-1, "axisID INPUT must be defined", __FUNCTION__);

  }
  int32_t *inSteps = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "useSteps");
 // char* auxData = getAttributeCache()->getRWPtr<char>(DOMAIN_CUSTOM, "auxiliaryDataset");

 if ((err = actuator_drv->initACT(*axID,(void *)NULL)) != 0){
    throw chaos::CFatalException(err, "Cannot init axis " + getDeviceID(), __FUNCTION__);
  }
 // use the JSON configuration parameter on device driver param
 if ((err = actuator_drv->configAxis(*axID,(void *)NULL)) != 0){
    throw chaos::CFatalException(err, "Cannot configure axis " + getDeviceID(), __FUNCTION__);
  }
  *homingDone = 0;
  this->updateAuxiliaryParameters();
  if ((err = actuator_drv->getState(*axID, &state_id, state_str)) != 0)
  {
   throw chaos::CFatalException(err, "Error getting the state of the actuator:"+state_str, __FUNCTION__);
  }

  *status_id = state_id;
  //notify change on status_id cached attribute

  if (actuator_drv->getHWVersion(*axID, device_hw) == 0)
  {
    SCCUDBG << "hardware found: \"" << device_hw << "\" axis:"<<*axID;
  }
  ::common::actuators::AbstractActuator::readingTypes readTyp;
  double tmp_float = 0.0F;
  const int32_t *tmpInt = getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType");
  readTyp = (::common::actuators::AbstractActuator::readingTypes)*tmpInt;
  if ((err = actuator_drv->getPosition(*axID, readTyp, &tmp_float)) == 0)
  {
    *o_position = tmp_float;
    *o_positionSP = tmp_float;
  }
  else
  {
    throw chaos::CFatalException(err, "Error getting initial position of the actuator", __FUNCTION__);
  }
const bool *doSoftHoming= getAttributeCache()->getROPtr<bool>(DOMAIN_INPUT, "AvoidPositionRecoverOnInit");

if (*doSoftHoming == 0)
{
  chaos::common::data::CDWUniquePtr loadedData(new CDataWrapper);
  loadedData=this->loadData("lastPosition");
  if (loadedData.get() != NULL)
  {
    double ps=loadedData->getDoubleValue("position");
   
    try
    {
      /* code */
       int32_t rT=loadedData->getInt32Value("readType");

    
       int32_t *tmpRead =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
	  //readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;    
      *tmpRead=rT;
      int32_t UI=loadedData->getInt32Value("useUI");
      std::string UIstr=ChaosToString(UI);
      if ((err= actuator_drv->setParameter(*axID,"useIU",UIstr))!=0)
      {
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    
    

    SCCUDBG << "ALEDEBUG loaded lastPosition: " <<  ps ;
    if ((err = actuator_drv->soft_homing(*axID,ps)) == 0) 
    {
      int32_t *o_kindof = getAttributeCache()->getRWPtr<int32_t>(DOMAIN_OUTPUT, "KindOfHomingDone");
      uint64_t *o_homingTime = getAttributeCache()->getRWPtr<uint64_t>(DOMAIN_OUTPUT, "LastHomingTime");
      *o_kindof=2;
      *o_homingTime =  chaos::common::utility::TimingUtil::getTimeStamp();


      }
     
    //Update lasthoming and update kindofHoming

  }
  else
    SCCUDBG << "ALEDEBUG no lastPosition to load" ;
}
  getAttributeCache()->setOutputDomainAsChanged();
  getAttributeCache()->setInputDomainAsChanged();
  getAttributeCache()->setCustomDomainAsChanged();
  metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo, boost::str(boost::format("Initialization of axis '%1%' done  ") % *axID ));
}

// Abstract method for the start of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitStart()
{
    SCCUDBG << "Starting";

}

// Abstract method for the stop of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitStop() 
{
  SCCUDBG << "Stop  "<<*axID;
  actuator_drv->stopMotion(*axID);

}

// Abstract method for the deinit of the control unit
void ::driver::actuator::SCActuatorControlUnit::unitDeinit()
{
  SCCUDBG << "Stop Motion "<<*axID;
  actuator_drv->stopMotion(*axID);
  //SCCUAPP << "Power off ";
  //actuator_drv->poweron(*axID, 0);
  SCCUDBG << "deinitializing "<<*axID;
  actuator_drv->deinitACT(*axID);
}

//! restore the control unit to snapshot
#define RESTORE_LAPP SCCUAPP << "[RESTORE-" << getCUID() << "] "
#define RESTORE_LDBG SCCUDBG << "[RESTORE-" << getCUID() << "] "

#define RESTORE_LERR SCCUERR << "[RESTORE-" << getCUID() << "] "

bool ::driver::actuator::SCActuatorControlUnit::unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache *const snapshot_cache) 
{
  uint64_t start_restore_time= chaos::common::utility::TimingUtil::getTimeStamp();

  try
  {

    RESTORE_LDBG << "Check if restore cache has the needed data";
    //check if in the restore cache we have all information we need
    RESTORE_LDBG << "Restore Check if  cache for position";
    if (!snapshot_cache->getSharedDomain(DOMAIN_INPUT).hasAttribute("position"))
    {
      RESTORE_LERR << " missing position to restore";
      return false;
    }
    RESTORE_LDBG << "Start the restore of the actuator";
    if (snapshot_cache == NULL)
    {
      RESTORE_LERR << "cache nulla";
      return false;
    }

    if (!snapshot_cache->getSharedDomain(DOMAIN_OUTPUT).hasAttribute("powerOn"))
    {
      RESTORE_LERR << " missing powerOn to restore";
      return false;
    }
 

    double restore_position_sp = *snapshot_cache->getAttributeValue(DOMAIN_INPUT, "position")->getValuePtr<double>();
    RESTORE_LDBG << "Restore Trying to set position at " << restore_position_sp;
    bool restore_power_sp = *snapshot_cache->getAttributeValue(DOMAIN_OUTPUT, "powerOn")->getValuePtr<bool>();
    RESTORE_LDBG << "Restore Trying to set power at " << restore_power_sp;

    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo, CHAOS_FORMAT("start restore \"%1%\" (axis %2%) to position %3% ", % getDeviceID() % *axID % restore_position_sp));

    //get actual state

    if (!setPowerOn(1))
    {
      metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError, CHAOS_FORMAT("Error applying power on during restore \"%1%\" (axis %2%) to position %3% ", % getDeviceID() % *axID % restore_position_sp));
      return false;
    }
    sleep(1);

    if (!setPosition(restore_position_sp))
    {
      metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError, CHAOS_FORMAT("Error SetPosition, restoring \"%1%\" (axis %2%) to position %3% ", % getDeviceID() % *axID % restore_position_sp));

      return false;
    }
    sleep(1);
    if (restore_power_sp == false)
    {
      if (!setPowerOn(0))
      {
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError, CHAOS_FORMAT("Error restoring power on during restore \"%1%\" (axis %2%) to value %3% ", % getDeviceID() % *axID % restore_power_sp));
        return false;
      }
    }
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo, CHAOS_FORMAT("Restored \"%1%\" (axis %2%) to position %3% in %4%", % getDeviceID() % *axID % restore_position_sp % restore_duration_in_ms));
    return true;
  }
  catch (CException &ex)
  {
    uint64_t restore_duration_in_ms = chaos::common::utility::TimingUtil::getTimeStamp() - start_restore_time;
    RESTORE_LERR << "[metric] Restore has fault in " << restore_duration_in_ms << " milliseconds";
    throw ex;
  }

  return false;
}

//-----------utility method for the restore operation---------

bool ::driver::actuator::SCActuatorControlUnit::setPosition(double val, bool sync)
{
  uint64_t cmd_id;
  bool result = true;
  ChaosUniquePtr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addDoubleValue(CMD_ACT_MM_OFFSET, val);
  //send command
  submitBatchCommand(CMD_ACT_MOVE_ABSOLUTE_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_AND_STACK);
  if (sync)
  {
    result = waitOnCommandID(cmd_id);
  }
  return result;
}

bool ::driver::actuator::SCActuatorControlUnit::setPowerOn(int32_t value, bool sync)
{
  uint64_t cmd_id;
  bool result = true;

  SCCUDBG << "LAUNCHING BATCH COMMAND setPowerOn " << value;
  ChaosUniquePtr<CDataWrapper> cmd_pack(new CDataWrapper());
  cmd_pack->addInt32Value(CMD_ACT_POWERON_VALUE, value);
  //send command
  submitBatchCommand(CMD_ACT_POWERON_ALIAS,
                     cmd_pack.release(),
                     cmd_id,
                     0,
                     50,
                     SubmissionRuleType::SUBMIT_AND_STACK);
  if (sync)
  {
    result = waitOnCommandID(cmd_id);
  }
  return result;
}

bool ::driver::actuator::SCActuatorControlUnit::waitOnCommandID(uint64_t cmd_id)
{
  ChaosUniquePtr<chaos::common::batch_command::CommandState> cmd_state;
  if(getState()!=chaos::CUStateKey::START){
		return true;
	}
  do
  {
    cmd_state = getStateForCommandID(cmd_id);
    if (!cmd_state.get())
      break;

    switch (cmd_state->last_event)
    {
    case BatchCommandEventType::EVT_QUEUED:
      SCCUDBG << cmd_id << " -> QUEUED";
      break;
    case BatchCommandEventType::EVT_RUNNING:
      SCCUDBG << cmd_id << " -> RUNNING";
      break;
    case BatchCommandEventType::EVT_WAITING:
      SCCUDBG << cmd_id << " -> WAITING";
      break;
    case BatchCommandEventType::EVT_PAUSED:
      SCCUDBG << cmd_id << " -> PAUSED";
      break;
    case BatchCommandEventType::EVT_KILLED:
      SCCUDBG << cmd_id << " -> KILLED";
      break;
    case BatchCommandEventType::EVT_COMPLETED:
      SCCUDBG << cmd_id << " -> COMPLETED";
      break;
    case BatchCommandEventType::EVT_FAULT:
      SCCUDBG << cmd_id << " -> FAULT";
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
