//
//  ChaosActuatorInterface.cpp
//
//  Created by alessandro D'Uffizi on 2/16/16.
//  Copyright (c) 2013 infn. All rights reserved.
//

#include "ChaosActuatorInterface.h"
using namespace chaos::driver::actuator;


#define PREPARE_OP_RET_INT_TIMEOUT(op,tim) \
actuator_oparams_t ret;\
actuator_iparams_t idata;\
message.opcode = op; \
message.inputData=(void*)&idata;\
idata.timeout=tim;\
message.inputDataLength=sizeof(actuator_iparams_t);\
message.resultDataLength=sizeof(actuator_oparams_t);\
message.resultData = (void*)&ret;\

/***************************/
#define READ_OP_FLOAT_PARAM_INT(op,ival,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
   accessor->send(&message); \
*pfval = ret.fvalue0; \
return ret.result; 

#define READ_OP_AX_FLOAT_PARAM_INT(op,ax,ival,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.ivalue=ival; \
   accessor->send(&message); \
*pfval = ret.fvalue0; \
return ret.result; 
/***************************/

#define WRITE_OP_INT_TIM_NORET(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
 accessor->send(&message,100);	
 
#define WRITE_OP_AX_INT_TIM_NORET(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.ivalue=ival; \
 accessor->send(&message,100);	


/***************************/
#define WRITE_OP_AX_INT_TIM(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.ivalue=ival; \
 accessor->send(&message,100);	\
return ret.result;

#define WRITE_OP_INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
 accessor->send(&message,100);	\
return ret.result;
/***************************/
#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.alarm_mask=ival;\
accessor->send(&message);\
return ret.result;

#define WRITE_OP_AX_64INT_TIM(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.alarm_mask=ival;\
accessor->send(&message);\
return ret.result;

/***************************/
#define WRITE_OP_AX_FLOAT_TIM(op,ax,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.fvalue0=fval;\
accessor->send(&message,100);	\
return ret.result;

#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval;\
accessor->send(&message,100);	\
return ret.result;
/***************************/
#define WRITE_OP_AX_2FLOAT_TIM(op,ax,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
idata.fvalue0=fval0;\
idata.fvalue1=fval1;\
accessor->send(&message);\
return ret.result;


#define WRITE_OP_2FLOAT_TIM(op,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval0;\
idata.fvalue1=fval1;\
accessor->send(&message);\
return ret.result;
/***************************/
#define READ_OP_AX_FLOAT_TIM(op,ax,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pfval = ret.fvalue0;\
return ret.result;


#define READ_OP_FLOAT_TIM(op,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval = ret.fvalue0;\
return ret.result;

/***************************/

#define READ_OP_AX_INT_TIM(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

/***************************/
#define READ_OP_AX_INT64_STRING_TIM(op,ax,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.alarm_mask;\
pstring = ret.str;\
return ret.result;
/***************************/
#define READ_OP_AX_INT_STRING_TIM(op,ax,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.ivalue;\
pstring = ret.str;\
return ret.result;

#define READ_OP_INT_STRING_TIM(op,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
pstring = ret.str;\
return ret.result;

/***************************/

#define WRITE_OP_AX_STRING_TIM(op,ax,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
strncpy(idata.str,(char*)pstring,MAX_STR_SIZE); \
accessor->send(&message);\
return ret.result;

#define WRITE_OP_STRING_TIM(op,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
strncpy(idata.str,(char*)pstring,MAX_STR_SIZE); \
accessor->send(&message);\
return ret.result;

/***************************/


#define WRITE_OP_STRING_STRING_TIM(op,pstring,pstring2,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
strncpy(idata.str,(char*)pstring,MAX_STR_SIZE); \
strncpy(idata.str2,(char*)pstring2,MAX_STR_SIZE); \
accessor->send(&message);\
return ret.result;

#define WRITE_OP_AX_STRING_STRING_TIM(op,ax,pstring,pstring2,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
strncpy(idata.str,(char*)pstring,MAX_STR_SIZE); \
strncpy(idata.str2,(char*)pstring2,MAX_STR_SIZE); \
idata.axis=ax;\
accessor->send(&message);\
return ret.result;
/***************************/

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_AX_INT_TIM(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

/***************************/
#define READ_OP_64INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;\
return ret.result;

#define READ_OP_AX_64INT_TIM(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.alarm_mask;\
return ret.result;
/***************************/


#define READ_OP_64INT_TIM_NORET(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;

#define READ_OP_AX_64INT_TIM_NORET(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pival = ret.alarm_mask;
/***************************/

#define READ_OP_2FLOAT_TIM(op,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval0 = ret.fvalue0;\
*pfval1 = ret.fvalue1;\
return ret.result;

#define READ_OP_AX_2FLOAT_TIM(op,ax,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
*pfval0 = ret.fvalue0;\
*pfval1 = ret.fvalue1;\
return ret.result;
/***************************/

#define WRITE_OP_TIM(op,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
return ret.result;


#define WRITE_OP_AX_TIM(op,ax,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.axis=ax;\
accessor->send(&message);\
return ret.result;
/***************************/

int ChaosActuatorInterface::init(void*d){
    WRITE_OP_STRING_TIM(OP_INIT,(char*)d,0);
}
int ChaosActuatorInterface::configAxis(void*d){
    DPRINT("ALEDEBUG CONFIG AXIS %s",(char*)d);
    WRITE_OP_STRING_TIM(OP_CONFIGAXIS,(char*)d,0);
}

int ChaosActuatorInterface::deinit(int32_t axisID){
    WRITE_OP_AX_TIM(OP_DEINIT,axisID,0);

}

int ChaosActuatorInterface::setTimeout(int32_t axisID,uint64_t timeo_ms) {
    WRITE_OP_AX_TIM(OP_SET_TIMEOUT,axisID,timeo_ms);
    
}

int ChaosActuatorInterface::getTimeout(int32_t axisID,uint64_t* timeo_ms) {
    READ_OP_AX_64INT_TIM(OP_GET_TIMEOUT,axisID,timeo_ms,0);
}

int ChaosActuatorInterface::getPosition(int32_t axisID,::common::actuators::AbstractActuator::readingTypes readingType,double *deltaPosition_mm) {
    READ_OP_AX_FLOAT_PARAM_INT(OP_GET_POSITION,axisID,readingType,deltaPosition_mm,0);
}

int ChaosActuatorInterface::resetAlarms(int32_t axisID,uint64_t alrm){
    WRITE_OP_AX_64INT_TIM(OP_RESET_ALARMS,axisID,alrm,0);
}

int ChaosActuatorInterface::getAlarms(int32_t axisID,uint64_t*alrm,std::string& desc){
    READ_OP_AX_INT64_STRING_TIM(OP_GET_ALARMS,axisID,alrm,desc,0);
}

int ChaosActuatorInterface::moveRelativeMillimeters(int32_t axisID,double mm) {
    float param=(float) mm;
    WRITE_OP_AX_FLOAT_TIM(OP_MOVE_RELATIVE_MM,axisID,param,0);
    
}
int ChaosActuatorInterface::moveAbsoluteMillimeters(int32_t axisID,double mm) {
    float param=(float) mm;
    WRITE_OP_AX_FLOAT_TIM(OP_MOVE_ABSOLUTE_MM,axisID,param,0);
    
}

int ChaosActuatorInterface::stopMotion(int32_t axisID){
    WRITE_OP_AX_TIM(OP_STOP_MOTION,axisID,0);
}

int ChaosActuatorInterface::homing(int32_t axisID,homingType mode){
    WRITE_OP_AX_INT_TIM(OP_HOMING,axisID,mode,0);
}


int ChaosActuatorInterface::poweron(int32_t axisID,int on){
    WRITE_OP_AX_INT_TIM(OP_POWERON,axisID,on,0);

}

int ChaosActuatorInterface::getState(int32_t axisID,int* state,std::string& desc){
    READ_OP_AX_INT_STRING_TIM(OP_GET_STATE,axisID, state, desc,0);
    //WRITE_OP_TIM(OP_GET_STATE,0);

}


int ChaosActuatorInterface::sendDataset(std::string& dataset){
    int state;
    READ_OP_INT_STRING_TIM(OP_SENDDATASET, &state, dataset,0);
}

int ChaosActuatorInterface::getSWVersion(int32_t axisID,std::string& ver){
    int state;
    READ_OP_AX_INT_STRING_TIM(OP_GET_SWVERSION,axisID, &state, ver,0);
}

int ChaosActuatorInterface::getHWVersion(int32_t axisID,std::string&ver){
    int state;
    READ_OP_AX_INT_STRING_TIM(OP_GET_HWVERSION,axisID, &state, ver,0);

}


uint64_t ChaosActuatorInterface::getFeatures() {
    uint64_t feats=0;
    READ_OP_64INT_TIM_NORET(OP_GET_FEATURE,&feats,0);
    return feats;
}
int ChaosActuatorInterface::setParameter(int32_t axisID,const std::string parName,const std::string value) {
	WRITE_OP_STRING_STRING_TIM(OP_SETPARAMETER,parName.c_str(),value.c_str(),0);
}






