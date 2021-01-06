/*
 *	Power supply base for DD
 *	!CHAOS
 *	Created by Andrea Michelotti
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
#include "ChaosActuatorExternalDD.h"

#include <string>
#include <boost/regex.hpp>
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include "ChaosActuatorExternalDD.h"
#include <driver/powersupply/core/ChaosPowerSupplyInterface.h>
using namespace chaos::driver::actuator;


/*
 *	ChaosActuatorExternalDD.cpp
 *
 *	!CHAOS
 *	Created by bisegni.
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
#undef DEBUG
#include "ChaosActuatorExternalDD.h"

#include <chaos/common/global.h>
#include <chaos/common/chaos_types.h>
#include <chaos/common/data/CDataVariant.h>
#undef INFO
#undef ERR
#undef DBG
#define INFO INFO_LOG(ChaosActuatorExternalDD)
#define ERR ERR_LOG(ChaosActuatorExternalDD)
#define DBG DBG_LOG(ChaosActuatorExternalDD)

using namespace chaos::driver::powersupply;
using namespace chaos::common::data;
using namespace chaos::cu::driver_manager::driver;
OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(ChaosActuatorExternalDD, 1.0.0, chaos::driver::actuator::ChaosActuatorExternalDD)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::ChaosActuatorExternalDD, http_address/dnsname:port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

//register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(chaos::driver::actuator::ChaosActuatorExternalDD)
CLOSE_REGISTER_PLUGIN
//default constructor definition
ChaosActuatorExternalDD::ChaosActuatorExternalDD() {
	
}


ChaosActuatorExternalDD::~ChaosActuatorExternalDD() {}

void ChaosActuatorExternalDD::driverInit(const chaos::common::data::CDataWrapper& init_parameter) throw(chaos::CException) {
    INFO << init_parameter.getJSONString();
    client.driverInit(init_parameter);

}
void ChaosActuatorExternalDD::driverInit(const char*c)throw(chaos::CException) {

}

void ChaosActuatorExternalDD::driverDeinit() throw(chaos::CException) {
        client.driverDeinit();

}

int ChaosActuatorExternalDD::asyncMessageReceived(CDWUniquePtr message) {
    INFO << message->getJSONString();
    return 0;
}

#define WRITE_ERR_ON_CMD(r, c, m, d)\
ERR <<m<<" domain:"<<d;

#define RETURN_ERROR(r, c, m, d)\
WRITE_ERR_ON_CMD(r, c, m, d)\
return err;

#define SEND_REQUEST(c, r,a)\
int err;\
if(err=client.sendRawRequest(ChaosMoveOperator(r), a)) {\
WRITE_ERR_ON_CMD(c, -1, "Timeout waiting answer from remote driver", __PRETTY_FUNCTION__);\
} else {\
if(response->hasKey("err")) {\
if(response->isInt32Value("err") == false)  {\
WRITE_ERR_ON_CMD(c, -3, "'err' key need to be an int32 value", __PRETTY_FUNCTION__);\
} else {\
err = response->getInt32Value("err");\
}\
} else {\
WRITE_ERR_ON_CMD(c, -2, "'err' key not found on external driver return package", __PRETTY_FUNCTION__);\
}\
}

#define SEND_REQUEST_OPC(opc, r,a) \
int err;{\
if((err=client.sendOpcodeRequest(opc,ChaosMoveOperator(r),a))) {\
WRITE_ERR_ON_CMD(err, -1, "Error from from remote driver", __PRETTY_FUNCTION__);\
}else {\
    if(response.get()==NULL){ERR<<"EMPTY RESPONSE";return -1;}\
    if(response->hasKey("err")) {\
    if(response->isInt32Value("err") == false)  {\
    WRITE_ERR_ON_CMD(c, -3, "'err' key need to be an int32 value", __PRETTY_FUNCTION__);\
    } else {\
    err = response->getInt32Value("err");\
    }\
    } else {\
    WRITE_ERR_ON_CMD(c, -2, "'err' key not found on external driver return package", __PRETTY_FUNCTION__);\
    }}\
if(response.get()==NULL){ERR<<"EMPTY RESPONSE";return -1;}\
}

#define CHECK_KEY_AND_TYPE_IN_RESPONSE(r, k, t, e1, e2)\
if(!r->hasKey(k)) {\
std::string es1 = CHAOS_FORMAT("'%1%' key is mandatory in remote driver response",%k);\
RETURN_ERROR(cmd, e1, es1.c_str(), __PRETTY_FUNCTION__);\
} else if(!r->t(k)) {\
std::string es2 = CHAOS_FORMAT("'%1%' key in remote driver response need to be int32 value", %k);\
RETURN_ERROR(cmd, e2, es2.c_str(), __PRETTY_FUNCTION__);\
}

#define CHECK_KEY_IN_RESPONSE(r, k, e1)\
if(!r->hasKey(k)) {\
std::string es1 = CHAOS_FORMAT("'%1%' key is mandatory in remote driver response",%k);\
RETURN_ERROR(cmd, e1, es1.c_str(), __PRETTY_FUNCTION__);\
}

int ChaosActuatorExternalDD::getParameter(int axisID,std::string parName,std::string &resultString) {
	     boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr data_pack(new CDataWrapper());
	data_pack->addInt32Value("axisID", axisID);
	data_pack->addStringValue("parameter_name",parName);
	SEND_REQUEST_OPC("get_parameter", data_pack, response);
	CHECK_KEY_IN_RESPONSE(response, "parameter_value", -2);
    resultString=response->getVariantValue("parameter_value").asString();
    return err;

}


int ChaosActuatorExternalDD::setParameter(int axisID,std::string parName,std::string parValue) {
 	     boost::mutex::scoped_lock lock(io_mux);

	 CDWShrdPtr response;
    	CDWUniquePtr get_para_pack(new CDataWrapper());
    	get_para_pack->addInt32Value("axisID", axisID);
	get_para_pack->addStringValue("parameter_name",parName);
	get_para_pack->addStringValue("parameter_value",parValue);
    	SEND_REQUEST_OPC("set_parameter", get_para_pack, response);
    	return err;
}

int ChaosActuatorExternalDD::getPosition( int32_t axisID, ::common::actuators::AbstractActuator::readingTypes readingType, double *deltaPosition) {
	     boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("readingType", readingType);

    SEND_REQUEST_OPC("get_pos", para_pack, response);
	CHECK_KEY_IN_RESPONSE(response, "value", -1);
	*deltaPosition=response->getVariantValue("value").asDouble();
	 return err;
}

int ChaosActuatorExternalDD::resetAlarms( int32_t axisID, uint64_t alrm) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt64Value("alarms",alrm);
	SEND_REQUEST_OPC("rst_alarm",para_pack,response);
	
    return err;


}

int ChaosActuatorExternalDD::hardreset( int32_t axisID, bool mode) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("mode",(int) mode);
	SEND_REQUEST_OPC("hard_reset",para_pack,response);
	
    return err;
}



int ChaosActuatorExternalDD::listParameters( std::string &dataset) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr data_pack(new CDataWrapper());
	SEND_REQUEST_OPC("listParameters", data_pack, response);
	CHECK_KEY_IN_RESPONSE(response, "dataset", -2);
    dataset=response->getVariantValue("dataset").asString();
    return err;
	
}

int ChaosActuatorExternalDD::getAlarms( int32_t axisID, uint64_t *alrm, std::string &desc) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("get_alarm", para_pack, response);
   
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    if(response->hasKey("description")){
	    desc=response->getStringValue("description");

    }
    *alrm = response->getInt32Value("value");
    return err;
}

int ChaosActuatorExternalDD::moveRelative( int32_t axisID, double delta) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addDoubleValue("value", delta);
	SEND_REQUEST_OPC("mov_rel", para_pack, response);
    return err;
}

int ChaosActuatorExternalDD::moveAbsolute( int32_t axisID, double setPos) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addDoubleValue("value", setPos);
	SEND_REQUEST_OPC("mov_abs", para_pack, response);
    return err;
}


int ChaosActuatorExternalDD::stopMotion( int32_t axisID) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("stopMotion", para_pack, response);
    return err;
}

int ChaosActuatorExternalDD::homing( int32_t axisID, ::common::actuators::AbstractActuator::homingType mode) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("mode", mode);
	SEND_REQUEST_OPC("homing", para_pack, response);
    return err;
}
int ChaosActuatorExternalDD::poweron( int32_t axisID, int on) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("on", on);
    DBG<<"poweron "<<axisID;

	SEND_REQUEST_OPC("poweron", para_pack, response);
    return err;
}

int ChaosActuatorExternalDD::getState( int32_t axisID, int *state, std::string &desc) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);

	SEND_REQUEST_OPC("get_state", para_pack, response);
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    if(response->hasKey("description")){
        desc=response->getStringValue("description");
    }
    *state = response->getInt32Value("value");
    return err;
}
int ChaosActuatorExternalDD::getSWVersion( int32_t axisID, std::string &version) {
	CHAOS_ASSERT(false);
    return 0;
}

int ChaosActuatorExternalDD::initACT(int axisID,void*ini){
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
	try {
		if(ini){
		CDWUniquePtr para_pack(new CDataWrapper());
		para_pack->addInt32Value("axisID", axisID);
		
		SEND_REQUEST_OPC("init", para_pack, response);
			return err;
		}
		return 0;
	} catch(...){

	}
	return -1;
}

int ChaosActuatorExternalDD::getHWVersion( int32_t axisID, std::string &version) {
	CDWShrdPtr response;
    CDWUniquePtr init_pack(new CDataWrapper());
	init_pack->addInt32Value("axisID", axisID);
    DBG<<"getHWVersion "<<axisID;
    SEND_REQUEST_OPC("get_hw_ver", init_pack, response);
	
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    version = response->getVariantValue("value").asString();
    return err;
}

int ChaosActuatorExternalDD::configAxis( int axisID,void *configuration) {
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);

    if(configuration){
	    para_pack->addStringValue("configString",(char*)configuration);
    } else {
        para_pack->addStringValue("configString","");

    }
	SEND_REQUEST_OPC("configAxis", para_pack, response);
    return err;

}

uint64_t ChaosActuatorExternalDD::getFeatures(){
	CDWShrdPtr response;
	CDWUniquePtr init_pack(new CDataWrapper());
    	SEND_REQUEST_OPC("get_feature", init_pack, response);
    	CHECK_KEY_IN_RESPONSE(response, "value", -1);
    	return response->getVariantValue("value").asUInt64();

}

int ChaosActuatorExternalDD::setTimeout( int32_t axisID, uint64_t timeo_ms) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt64Value("timeout",timeo_ms);
	SEND_REQUEST_OPC("set_timeout",para_pack,response);
	
    return err;
}
int ChaosActuatorExternalDD::getTimeout( int32_t axisID, uint64_t *timeo_ms) {
	CDWShrdPtr response;
    	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("get_timeout", para_pack, response);
	CHECK_KEY_IN_RESPONSE(response, "value", -1);
	*timeo_ms=response->getVariantValue("value").asUInt64();
	return err;
}

int ChaosActuatorExternalDD::deinitACT( int32_t axisID)
{
	boost::mutex::scoped_lock lock(io_mux);

	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("deinit", para_pack, response);
	return err;
}



int ChaosActuatorExternalDD::setAdditive( bool isAdditive) {
	CHAOS_ASSERT(false);
    return 0;
}         
        
//! Execute a command
MsgManagmentResultType::MsgManagmentResult ChaosActuatorExternalDD::execOpcode(DrvMsgPtr cmd) {
         ERR<<"Opcode not supported:"<<cmd->opcode;
    
}
