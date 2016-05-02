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

#define READ_OP_FLOAT_PARAM_INT(op,ival,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
   accessor->send(&message); \
*pfval = ret.fvalue0; \
return ret.result; 

#define WRITE_OP_INT_TIM_NORET(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
 accessor->send(&message,100);	




#define WRITE_OP_INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.ivalue=ival; \
 accessor->send(&message,100);	\
return ret.result;

#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.alarm_mask=ival;\
accessor->send(&message);\
return ret.result;

#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata.fvalue0=fval;\
accessor->send(&message,100);	\
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


#define WRITE_OP_STRING_TIM(op,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
strncpy(idata.str,(char*)pstring,MAX_STR_SIZE); \
accessor->send(&message);\
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

#define WRITE_OP_BASE(op) \
PREPARE_OP_BASE(op); \
accessor->send(&message);\
return 0;

int ChaosActuatorInterface::init(void*d){
    WRITE_OP_STRING_TIM(OP_INIT,(char*)d,0);
}

int ChaosActuatorInterface::deinit(){
    WRITE_OP_TIM(OP_DEINIT,0);

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

int ChaosActuatorInterface::setAcceleration(double acceleration_mm_per_sec2) {
       float param=(float) acceleration_mm_per_sec2;
    WRITE_OP_FLOAT_TIM(OP_SET_SPEED,param,0);
}

int ChaosActuatorInterface::setAdditive(bool isAdditive)
{
    WRITE_OP_INT_TIM(OP_SET_ADDITIVE,isAdditive,0);
}

int ChaosActuatorInterface::setReferenceBase(int32_t referenceBase)
{
    WRITE_OP_INT_TIM(OP_SET_REFERENCE,referenceBase,0);
}

int ChaosActuatorInterface::setMovement(int32_t movement)
{
    WRITE_OP_INT_TIM(OP_SET_MOVEMENT,movement,0);
}
int ChaosActuatorInterface::getPosition(::common::actuators::AbstractActuator::readingTypes readingType,double *deltaPosition_mm) {
    READ_OP_FLOAT_PARAM_INT(OP_GET_POSITION,readingType,deltaPosition_mm,0);
}

int ChaosActuatorInterface::resetAlarms(uint64_t alrm){
    WRITE_OP_64INT_TIM(OP_RESET_ALARMS,alrm,0);
}

int ChaosActuatorInterface::getAlarms(uint64_t*alrm,std::string& desc){
    READ_OP_64INT_TIM(OP_GET_ALARMS,alrm,0);
}

int ChaosActuatorInterface::moveRelativeMillimeters(double mm) {
    float param=(float) mm;
    WRITE_OP_FLOAT_TIM(OP_MOVE_RELATIVE_MM,param,0);
    
}
int ChaosActuatorInterface::moveAbsoluteMillimeters(double mm) {
    float param=(float) mm;
    WRITE_OP_FLOAT_TIM(OP_MOVE_ABSOLUTE_MM,param,0);
    
}

int ChaosActuatorInterface::stopMotion(){
    WRITE_OP_TIM(OP_STOP_MOTION,0);
}

int ChaosActuatorInterface::homing(homingType mode){
    WRITE_OP_INT_TIM(OP_HOMING,mode,0);
}


int ChaosActuatorInterface::poweron(uint32_t timeo_ms){
    WRITE_OP_TIM(OP_POWERON,timeo_ms);

}

int ChaosActuatorInterface::getState(int* state,std::string& desc){
    READ_OP_INT_STRING_TIM(OP_GET_STATE, state, desc,0);
    //WRITE_OP_TIM(OP_GET_STATE,0);

}


int ChaosActuatorInterface::getSWVersion(std::string& ver){
    int state;
    READ_OP_INT_STRING_TIM(OP_GET_SWVERSION, &state, ver,0);
}

int ChaosActuatorInterface::getHWVersion(std::string&ver){
    int state;
    READ_OP_INT_STRING_TIM(OP_GET_HWVERSION, &state, ver,0);

}


uint64_t ChaosActuatorInterface::getFeatures() {
    uint64_t feats=0;
    READ_OP_64INT_TIM_NORET(OP_GET_FEATURE,&feats,0);
    return feats;
}

int ChaosActuatorInterface::setTrapezoidalProfile(double, double, bool, int32_t, int32_t) {
return 0;
}







