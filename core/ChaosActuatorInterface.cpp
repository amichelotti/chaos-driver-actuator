//
//  ChaosActuatorInterface.cpp
//
//  Created by andrea michelotti on 10/24/13.
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

#define WRITE_OP_INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival;\
 accessor->send(&message,100);			\
return ret.result;

#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.alarm_mask=ival;\
accessor->send(&message);\
return ret.result;

#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval;\
 accessor->send(&message,100);			\
return ret.result;

#define WRITE_OP_2FLOAT_TIM(op,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval0;\
idata.fvalue1=fval1;\
accessor->send(&message);\
return ret.result;

#define READ_OP_FLOAT_TIM(op,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval = ret.fvalue0;\
return ret.result;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_INT_STRING_TIM(op,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
pstring = ret.str;\
return ret.result;

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.ivalue;\
return ret.result;

#define READ_OP_64INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;\
return ret.result;

#define READ_OP_64INT_TIM_NORET(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret.alarm_mask;


#define READ_OP_2FLOAT_TIM(op,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pfval0 = ret.fvalue0;\
*pfval1 = ret.fvalue1;\
return ret.result;

#define WRITE_OP_TIM(op,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
return ret.result;




int ChaosActuatorInterface::resetAlarms(uint64_t alrm){
    WRITE_OP_64INT_TIM(OP_RESET_ALARMS,alrm,0);
}

int ChaosActuatorInterface::getAlarms(uint64_t*alrm){
    READ_OP_64INT_TIM(OP_GET_ALARMS,alrm,0);
}


int ChaosActuatorInterface::poweron(uint32_t timeo_ms){
    WRITE_OP_TIM(OP_POWERON,timeo_ms);

}

int ChaosActuatorInterface::getState(int* state,std::string& desc){
    READ_OP_INT_STRING_TIM(OP_GET_STATE, state, desc,0);

}

int ChaosActuatorInterface::init(){
    WRITE_OP_TIM(OP_INIT,0);
}

int ChaosActuatorInterface::deinit(){
    WRITE_OP_TIM(OP_DEINIT,0);

}
int ChaosActuatorInterface::getSWVersion(std::string& ver){
    int state;
    READ_OP_INT_STRING_TIM(OP_GET_SWVERSION, &state, ver,0);
}

int ChaosActuatorInterface::getHWVersion(std::string&ver){
    int state;
    READ_OP_INT_STRING_TIM(OP_GET_HWVERSION, &state, ver,0);

}

int ChaosActuatorInterface::getAlarmDesc(uint64_t *desc){
    READ_OP_64INT_TIM(OP_GET_ALARM_DESC,desc,0);

}

int ChaosActuatorInterface::stopMotion(){
    WRITE_OP_TIM(OP_STOP_MOTION,0);
}

bool ChaosActuatorInterface::homing(int minutes, int mode){
    WRITE_OP_INT_TIM(OP_HOMING,mode,0);
}

int getPosition(::common::actuators::AbstractActuator::readingTypes readingType,double& deltaPosition_mm) {
    
}
uint64_t ChaosActuatorInterface::getFeatures() {
    uint64_t feats=0;
    READ_OP_64INT_TIM_NORET(OP_GET_FEATURE,&feats,0);
    return feats;
}



int ChaosActuatorInterface::moveRelativeMillimeters(double mm) {
    float param=(float) mm;
    WRITE_OP_FLOAT_TIM(OP_MOVE_RELATIVE_MM,param,0);
    
}

int ChaosActuatorInterface::setTimeout(uint64_t timeo_ms) {
    WRITE_OP_TIM(OP_SET_TIMEOUT,timeo_ms);
    
}

int ChaosActuatorInterface::getTimeout(uint64_t* timeo_ms) {
    READ_OP_64INT_TIM(OP_GET_TIMEOUT,timeo_ms,0);
}
int ChaosActuatorInterface::setSpeed(double speed_mm_per_sec) {
        float param=(float) speed_mm_per_sec;
    WRITE_OP_FLOAT_TIM(OP_SET_SPEED,param,0);

    
}