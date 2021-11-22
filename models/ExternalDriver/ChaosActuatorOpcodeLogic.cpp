/*
 *	ChaosActuatorOpcodeLogic.cpp
 *
 *	!CHAOS
 *	Created by D'Uffizi.
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

#include "ChaosActuatorOpcodeLogic.h"
#include "../../core/ChaosActuatorInterface.h"

#include <chaos/common/global.h>
#include <chaos/common/chaos_types.h>
#include <chaos/common/data/CDataVariant.h>
#undef INFO
#undef ERR
#undef DBG
#define INFO INFO_LOG(ChaosActuatorOpcodeLogic)
#define ERR ERR_LOG(ChaosActuatorOpcodeLogic)
#define DBG DBG_LOG(ChaosActuatorOpcodeLogic)
#define DBGO DBG_LOG(ChaosActuatorOpcodeLogic) <<"- "<<cmd->id<<" -"

using namespace chaos::driver::actuator;
using namespace chaos::common::data;
using namespace chaos::cu::driver_manager::driver;

ChaosActuatorOpcodeLogic::ChaosActuatorOpcodeLogic(chaos::cu::driver_manager::driver::RemoteIODriverProtocol *_remote_driver):
OpcodeExternalCommandMapper(_remote_driver){}

ChaosActuatorOpcodeLogic::~ChaosActuatorOpcodeLogic() {}

void ChaosActuatorOpcodeLogic::driverInit(const chaos::common::data::CDataWrapper& init_parameter) throw(chaos::CException) {
    INFO << init_parameter.getJSONString();
}

void ChaosActuatorOpcodeLogic::driverDeinit() throw(chaos::CException) {
    
}

int ChaosActuatorOpcodeLogic::asyncMessageReceived(CDWUniquePtr message) {
    INFO << message->getJSONString();
    return 0;
}



#define WRITE_ERR_ON_CMD(r, c, m, d)\
cmd->ret = c;\
snprintf(cmd->err_msg, 255, "%s", m);\
snprintf(cmd->err_dom, 255, "%s", d);

#define RETURN_ERROR(r, c, m, d)\
WRITE_ERR_ON_CMD(r, c, m, d)\
return cmd->ret;

#define SEND_REQUEST(c, r,a)\
if(sendRawRequest(ChaosMoveOperator(r), a)) {\
WRITE_ERR_ON_CMD(c, -1, "Timeout waiting answer from remote driver", __PRETTY_FUNCTION__);\
} else {\
if(response->hasKey("err")) {\
if(response->isInt32Value("err") == false)  {\
WRITE_ERR_ON_CMD(c, -3, "'err' key need to be an int32 value", __PRETTY_FUNCTION__);\
} else {\
c->ret = response->getInt32Value("err");\
}\
} else {\
WRITE_ERR_ON_CMD(c, -2, "'err' key not found on external driver return package", __PRETTY_FUNCTION__);\
}\
}

#define SEND_REQUEST_OPC(opc,c, r,a) {\
    int err;\
    fillMessageWithDeviceParam(c,*r); \
try { \
if((err=sendOpcodeRequest(opc,ChaosMoveOperator(r),a))) {\
WRITE_ERR_ON_CMD(err, -1, "Error from from remote driver", __PRETTY_FUNCTION__);\
}else {\
    if(response->hasKey("err")) {\
    if(response->isInt32Value("err") == false)  {\
    WRITE_ERR_ON_CMD(c, -3, "'err' key need to be an int32 value", __PRETTY_FUNCTION__);\
    } else {\
    c->ret = response->getInt32Value("err");\
    }\
    } else {\
    WRITE_ERR_ON_CMD(c, -2, "'err' key not found on external driver return package", __PRETTY_FUNCTION__);\
    }}\
	}\
    catch (...) \
    {	\
DBG << "Exception catched in send request opc" ; \
WRITE_ERR_ON_CMD(err, -6, "Exception from remote driver", __PRETTY_FUNCTION__);\
    }\
}

#define CHECK_KEY_AND_TYPE_IN_RESPONSE(r, k, t, e1, e2)\
if(!r->hasKey(k)) {\
std::string es1 = CHAOS_FORMAT("'%1%' key is mandatory in remote driver response",%k);\
RETURN_ERROR(cmd, e1, es1.c_str(), __PRETTY_FUNCTION__);\
} else if(!r->t(k)) {\
std::string es2 = CHAOS_FORMAT("'%1%' key in remote driver response need to be the right kind of value", %k);\
RETURN_ERROR(cmd, e2, es2.c_str(), __PRETTY_FUNCTION__);\
}

#define CHECK_KEY_IN_RESPONSE(r, k, e1)\
if(!r->hasKey(k)) {\
std::string es1 = CHAOS_FORMAT("'%1%' key is mandatory in remote driver response",%k);\
RETURN_ERROR(cmd, e1, es1.c_str(), __PRETTY_FUNCTION__);\
}

/*
int ChaosActuatorOpcodeLogic::sendInit(DrvMsgPtr cmd) {
     CDWShrdPtr response;

    CDWUniquePtr init_pack(new CDataWrapper());

    cmd->ret =  0;
    SEND_REQUEST_OPC("init",cmd, init_pack, response);

    if(response.get()){DBG << response->getJSONString();}

    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::sendDeinit(DrvMsgPtr cmd) {
     CDWShrdPtr response;

    CDWUniquePtr init_pack(new CDataWrapper());

    cmd->ret =  0;
    SEND_REQUEST_OPC("deinit",cmd, init_pack, response);

    if(response.get()){DBG << response->getJSONString();}

    return cmd->ret;
}
*/

int ChaosActuatorOpcodeLogic::getParameter(DrvMsgPtr cmd,int axisID,std::string parName,std::string &resultString) {
	CDWShrdPtr response;
    CDWUniquePtr data_pack(new CDataWrapper());
	data_pack->addInt32Value("axisID", axisID);
	data_pack->addStringValue("parameter_name",parName);
	SEND_REQUEST_OPC("get_parameter",cmd, data_pack, response);
    if(cmd->ret) {return cmd->ret;}
	CHECK_KEY_IN_RESPONSE(response, "parameter_value", -2);
    resultString=response->getVariantValue("parameter_value").asString();
    return cmd->ret;

}


int ChaosActuatorOpcodeLogic::setParameter(DrvMsgPtr cmd,int axisID,std::string parName,std::string parValue) {
 	CDWShrdPtr response;
    	CDWUniquePtr get_para_pack(new CDataWrapper());
    	get_para_pack->addInt32Value("axisID", axisID);
	get_para_pack->addStringValue("parameter_name",parName);
	get_para_pack->addStringValue("parameter_value",parValue);
    	SEND_REQUEST_OPC("set_parameter",cmd, get_para_pack, response);
    	return cmd->ret;
}

int ChaosActuatorOpcodeLogic::getPosition(DrvMsgPtr cmd, int32_t axisID, ::common::actuators::AbstractActuator::readingTypes readingType, double *deltaPosition) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("readingType", readingType);

    SEND_REQUEST_OPC("get_pos",cmd, para_pack, response);
    if(cmd->ret) {return cmd->ret;}
	CHECK_KEY_IN_RESPONSE(response, "value", -1);
	*deltaPosition=response->getVariantValue("value").asDouble();
	 return cmd->ret;
}

int ChaosActuatorOpcodeLogic::resetAlarms(DrvMsgPtr cmd, int32_t axisID, uint64_t alrm) {
	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt64Value("alarms",alrm);
	SEND_REQUEST_OPC("rst_alarm",cmd,para_pack,response);
	
    return cmd->ret;


}

int ChaosActuatorOpcodeLogic::hardreset(DrvMsgPtr cmd, int32_t axisID, bool mode) {
	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("mode",(int) mode);
	SEND_REQUEST_OPC("hard_reset",cmd,para_pack,response);
	
    return cmd->ret;
}



int ChaosActuatorOpcodeLogic::listParameters(DrvMsgPtr cmd, std::string &dataset) {
	CDWShrdPtr response;
    CDWUniquePtr data_pack(new CDataWrapper());
	SEND_REQUEST_OPC("listParameters",cmd, data_pack, response);
    if(cmd->ret) {return cmd->ret;}
	CHECK_KEY_IN_RESPONSE(response, "dataset", -2);
    dataset=response->getVariantValue("dataset").asString();
    return cmd->ret;
	
}

int ChaosActuatorOpcodeLogic::getAlarms(DrvMsgPtr cmd, int32_t axisID, uint64_t *alrm, std::string &desc) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("get_alarm",cmd, para_pack, response);
   
    if(cmd->ret) {return cmd->ret;}
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    if(response->hasKey("description")){
	    desc=response->getStringValue("description");

    }
    *alrm = response->getInt32Value("value");
    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::moveRelative(DrvMsgPtr cmd, int32_t axisID, double delta) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addDoubleValue("value", delta);
	SEND_REQUEST_OPC("mov_rel",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::moveAbsolute(DrvMsgPtr cmd, int32_t axisID, double setPos) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addDoubleValue("value", setPos);
	SEND_REQUEST_OPC("mov_abs",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}


int ChaosActuatorOpcodeLogic::stopMotion(DrvMsgPtr cmd, int32_t axisID) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("stopMotion",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}
int ChaosActuatorOpcodeLogic::soft_homing(cu::driver_manager::driver::DrvMsgPtr cmd,int32_t axisID,double positionToSet)
{
    CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addDoubleValue("value", positionToSet);
	SEND_REQUEST_OPC("soft_homing",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::homing(DrvMsgPtr cmd, int32_t axisID, ::common::actuators::AbstractActuator::homingType mode) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("mode", mode);
	SEND_REQUEST_OPC("homing",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}
int ChaosActuatorOpcodeLogic::poweron(DrvMsgPtr cmd, int32_t axisID, int on) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt32Value("on", on);
    DBG<<"poweron "<<axisID;

	SEND_REQUEST_OPC("poweron",cmd, para_pack, response);
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::getState(DrvMsgPtr cmd, int32_t axisID, int *state, std::string &desc) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);

	SEND_REQUEST_OPC("get_state",cmd, para_pack, response);
    if(response.get()){DBG << response->getJSONString();}
    if(cmd->ret) {return cmd->ret;}
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    if(response->hasKey("description")){
        desc=response->getStringValue("description");
    }
    *state = response->getInt32Value("value");
    return cmd->ret;
}
int ChaosActuatorOpcodeLogic::getSWVersion(DrvMsgPtr cmd, int32_t axisID, std::string &version) {
	CHAOS_ASSERT(false);
    return 0;
}
int ChaosActuatorOpcodeLogic::getHWVersion(DrvMsgPtr cmd, int32_t axisID, std::string &version) {
	CDWShrdPtr response;
    CDWUniquePtr init_pack(new CDataWrapper());
	init_pack->addInt32Value("axisID", axisID);
    DBG<<"getHWVersion "<<axisID;
    SEND_REQUEST_OPC("get_hw_ver",cmd, init_pack, response);
	
    if(response.get()){DBG << response->getJSONString();}
    if(cmd->ret) {return cmd->ret;}
    CHECK_KEY_IN_RESPONSE(response, "value", -1);
    version = response->getVariantValue("value").asString();
    return cmd->ret;
}

int ChaosActuatorOpcodeLogic::configAxis(DrvMsgPtr cmd, void *configuration) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());

    if(configuration){
	    para_pack->addStringValue("configString",(char*)configuration);
    } else {
        para_pack->addStringValue("configString","");

    }
	SEND_REQUEST_OPC("configAxis",cmd, para_pack, response);
    return cmd->ret;

}

uint64_t ChaosActuatorOpcodeLogic::getFeatures(DrvMsgPtr cmd){
	CDWShrdPtr response;
	CDWUniquePtr init_pack(new CDataWrapper());
    	SEND_REQUEST_OPC("get_feature",cmd, init_pack, response);
    	if(response.get()){DBG << response->getJSONString();}
    	if(cmd->ret) {return cmd->ret;}
    	CHECK_KEY_IN_RESPONSE(response, "value", -1);
    	return response->getVariantValue("value").asUInt64();

}

int ChaosActuatorOpcodeLogic::setTimeout(DrvMsgPtr cmd, int32_t axisID, uint64_t timeo_ms) {
	CDWShrdPtr response;
    CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	para_pack->addInt64Value("timeout",timeo_ms);
	SEND_REQUEST_OPC("set_timeout",cmd,para_pack,response);
	
	if(response.get()){DBG << response->getJSONString();}
    return cmd->ret;
}
int ChaosActuatorOpcodeLogic::getTimeout(DrvMsgPtr cmd, int32_t axisID, uint64_t *timeo_ms) {
	CDWShrdPtr response;
    	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("get_timeout",cmd, para_pack, response);
    	if(response.get()){DBG << response->getJSONString();}
    	if(cmd->ret) {return cmd->ret;}
	CHECK_KEY_IN_RESPONSE(response, "value", -1);
	*timeo_ms=response->getVariantValue("value").asUInt64();
	return cmd->ret;
}

int ChaosActuatorOpcodeLogic::deinit(DrvMsgPtr cmd, int32_t axisID)
{
	CDWShrdPtr response;
	CDWUniquePtr para_pack(new CDataWrapper());
	para_pack->addInt32Value("axisID", axisID);
	SEND_REQUEST_OPC("deinit", cmd, para_pack, response);
	if (response.get()) { DBG << response->getJSONString(); }
	return cmd->ret;
}



int ChaosActuatorOpcodeLogic::setAdditive(DrvMsgPtr cmd, bool isAdditive) {
	CHAOS_ASSERT(false);
    return 0;
}
//! Execute a command
MsgManagmentResultType::MsgManagmentResult ChaosActuatorOpcodeLogic::execOpcode(DrvMsgPtr cmd) {
     boost::mutex::scoped_lock lock(io_mux);

    MsgManagmentResultType::MsgManagmentResult result = MsgManagmentResultType::MMR_EXECUTED;
    actuator_iparams_t *in = (actuator_iparams_t *)cmd->inputData;
    actuator_oparams_t *out = (actuator_oparams_t *)cmd->resultData;
    cmd->ret = 0;
    memset(cmd->err_msg, 0, 255);
    memset(cmd->err_dom, 0, 255);
 //   DBGO<<" START OPCODE:"<<cmd->opcode<<" timeo:"<<in->timeout;

    switch(cmd->opcode) {
        case OP_INIT:
			
            out->result = sendInit(cmd);
            break;
        case OP_DEINIT:
			//deinit(cmd, in->axis);
            out->result = sendDeinit(cmd);
            break;
        case OP_CONFIGAXIS:
            DBGO<< "Configuring with:"<<in->str<<" timeo:"<<in->timeout;
            out->result= configAxis(cmd,(void*) in->str);
            break;

        case OP_GET_POSITION:
            out->result = getPosition(cmd, in->axis,(::common::actuators::AbstractActuator::readingTypes)in->ivalue,&out->fvalue0);
      //      DBGO<< "Got Position :"<< out->fvalue0;
            break;

        case OP_RESET_ALARMS:
       //     DBGO<< "Reset alarms to:"<<in->alarm_mask << std::endl;
            out->result = resetAlarms(cmd, in->axis,in->alarm_mask);
            break;

        case OP_HARD_RESET:
        //    DBGO<< "HARD Reset in axis "<< in->axis << std::endl;
            out->result = hardreset(cmd,in->axis,in->ivalue);
            break;
 	case OP_GET_ALARMS:
             {
		std::string desc;
            out->result = getAlarms(cmd,in->axis, &out->alarm_mask,desc);
			strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
       //     DBGO<<"Got alarms to: "<<out->alarm_mask << desc;
            }
            break;
       
        case OP_MOVE_RELATIVE_MM:
	//    DBGO<< "Move relative offset: "<<in->fvalue0;
            out->result = moveRelative(cmd,in->axis,in->fvalue0);
            break;

        case OP_MOVE_ABSOLUTE_MM:
       //     DBGO<< "Move Absolute to position "<<in->fvalue0;
            out->result = moveAbsolute(cmd,in->axis, in->fvalue0);
            break;

        case OP_STOP_MOTION: //stop motion
            out->result = stopMotion(cmd, in->axis);
		//	DBGO<< "Stop Motion: result "<<out->result;
            break;

        case OP_HOMING: 
            out->result = this->homing(cmd,in->axis,  (::common::actuators::AbstractActuator::homingType)in->ivalue );
	// DBGO<< "Set homing, homing type: "<< in->ivalue << "result is " << out->result;
           break;
		case OP_POWERON:
     //   	DBGO<<"poweron" << in->axis;

            out->result = poweron(cmd, in->axis,in->ivalue);
	//		DBGO<<"Set Power" << in->ivalue <<" , result:"<< out->result;
            break;
		 case OP_GET_STATE:{
            std::string desc;
      //      DBGO<<"getState axis:" << in->axis;

            out->result = getState(cmd,in->axis, &out->ivalue,desc);
            strncpy(out->str,desc.c_str(),MAX_STR_SIZE);
      //      DBGO<<"Got State: "<<out->ivalue<<" \""<<desc<<"\"";
			break;
						   }
	    case OP_GET_SWVERSION:{
            std::string ver;

            out->result = getSWVersion(cmd, in->axis,ver);
     //       DBGO <<"Got SW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;
        }
            break;
        case OP_GET_HWVERSION:{
            std::string ver;
            out->result = getHWVersion(cmd,in->axis, ver);
     //       DBGO <<"Got HW Version:\""<<ver;
            strncpy(out->str,ver.c_str(),MAX_STR_SIZE);;
        }
            break;


        case OP_LISTPARAMETERS: {
	std::string dataset;
	out->result = this->listParameters(cmd,dataset);
        strncpy(out->str,dataset.c_str(),JSON_MAX_SIZE);
        }
        break;


        case OP_SETPARAMETER:
	out->result = this->setParameter(cmd,in->axis,in->str,in->str2);
//	DBGO << "Sending SetParameter  " << in->str2 <<"  on Parameter " << in->str <<" axis " <<in->axis<< " result " << out->result ;
        break;

        case OP_GETPARAMETER: {
                std::string tempString;
                out->result=this->getParameter(cmd,in->axis,in->str,tempString);
                strncpy(out->str,tempString.c_str(),JSON_MAX_SIZE);
      //          DBGO << "GetParameter asked for " << in->str << " received " <<out->str;
        }
        break;
       
       
        case OP_GET_FEATURE:{
            uint64_t feat=getFeatures(cmd);
            out->alarm_mask=feat;
      //      DBGO<<"Got Features:"<<feat;
        }
            break;
          
            default:
            ERR<<"Opcode not supported:"<<cmd->opcode;
	    break;
    }
  //  DBGO<<cmd->opcode<<" RETURNED:"<<result;

    return result;
}
