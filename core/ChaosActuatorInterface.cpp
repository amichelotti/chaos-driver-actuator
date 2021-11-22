//
//  ChaosActuatorInterface.cpp
//
//  Created by alessandro D'Uffizi on 2/16/16.
//  Copyright (c) 2013 infn. All rights reserved.
//

#include "ChaosActuatorInterface.h"
#include "ChaosActuatorDD.h"
using namespace chaos::driver::actuator;
#define SCCUAPP INFO_LOG(ChaosActuatorInterface) << "[" << owner << "]- "<<accessor->getMessageCount()<<"-"
#define SCCUDBG DBG_LOG(ChaosActuatorInterface) << "[" << owner << "]- "<<accessor->getMessageCount()<<"-"
#define SCCUERR ERR_LOG(ChaosActuatorInterface) << "[" << owner << "]- "<<accessor->getMessageCount()<<"-"


#define RETURN \
    {int tmp=ret->result;free(message.inputData);free(message.resultData);return tmp;}

#define SEND_AND_RETURN \
 accessor->send(&message);			\
RETURN

#define SEND_AND_RETURN_TIM(t) \
 accessor->send(&message,t);			\
RETURN



#define PREPARE_OP_RET_INT_TIMEOUT(op,tim) \
actuator_oparams_t* ret=(actuator_oparams_t*)calloc(1,sizeof(actuator_oparams_t));\
actuator_iparams_t* idata=(actuator_iparams_t*)calloc(1,sizeof(actuator_iparams_t));\
ret->result=DRV_BYPASS_DEFAULT_CODE;\
idata->timeout=tim;\
message.opcode = op; \
message.inputData=(void*)idata;\
message.inputDataLength=sizeof(actuator_iparams_t);\
message.resultDataLength=sizeof(actuator_oparams_t);\
message.resultData = (void*)ret;\

/***************************/
#define READ_OP_FLOAT_PARAM_INT(op,ival,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->ivalue=ival; \
ret->fvalue0=*pfval;\
accessor->send(&message,timeout); \
*pfval = ret->fvalue0; \
RETURN

#define READ_OP_AX_FLOAT_PARAM_INT(op,ax,ival,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->ivalue=ival; \
ret->fvalue0=*pfval;\
   accessor->send(&message,timeout); \
*pfval = ret->fvalue0; \
RETURN

/***************************/

#define WRITE_OP_INT_TIM_NORET(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->ivalue=ival; \
 accessor->send(&message,timeout);\
free(message.inputData);free(message.resultData); 

#define WRITE_OP_AX_INT_TIM_NORET(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->ivalue=ival; \
 accessor->send(&message,timeout);	\
free(message.inputData);free(message.resultData);

/***************************/
#define WRITE_OP_AX_INT_TIM(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->ivalue=ival; \
ret->result=0;\
SEND_AND_RETURN

#define WRITE_OP_INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->ivalue=ival; \
ret->result=0;\
SEND_AND_RETURN

/***************************/
#define WRITE_OP_64INT_TIM(op,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->alarm_mask=ival;\
ret->result=0;\
SEND_AND_RETURN

#define WRITE_OP_AX_64INT_TIM(op,ax,ival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->alarm_mask=ival;\
ret->result=0;\
SEND_AND_RETURN

/***************************/
#define WRITE_OP_AX_FLOAT_TIM(op,ax,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->fvalue0=fval;\
ret->result=0;\
SEND_AND_RETURN

#define WRITE_OP_FLOAT_TIM(op,fval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->fvalue0=fval;\
ret->result=0;\
SEND_AND_RETURN

/***************************/
#define WRITE_OP_AX_2FLOAT_TIM(op,ax,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
idata->fvalue0=fval0;\
idata->fvalue1=fval1;\
ret->result=0;\
SEND_AND_RETURN

#define WRITE_OP_2FLOAT_TIM(op,fval0,fval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->fvalue0=fval0;\
idata->fvalue1=fval1;\
ret->result=0;\
SEND_AND_RETURN

/***************************/
#define READ_OP_AX_FLOAT_TIM(op,ax,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
ret->fvalue0=*pfval;\
ret->result=0;\
accessor->send(&message,timeout);\
*pfval = ret->fvalue0;\
RETURN

#define READ_OP_FLOAT_TIM(op,pfval,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
ret->fvalue0=*pfval;\
ret->result=0;\
accessor->send(&message,timeout);\
*pfval = ret->fvalue0;\
RETURN

/***************************/

#define READ_OP_AX_INT_TIM(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
ret->fvalue0=*pfval;\
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
RETURN

#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
ret->ivalue=*pival;\
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
RETURN

/***************************/
#define READ_OP_AX_INT64_STRING_TIM(op,ax,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
ret->alarm_mask=*pival ;\
*ret->str=0;\
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;\
pstring = ret->str;\
RETURN
/***************************/
#define READ_OP_AX_INT_STRING_TIM(op,ax,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
*ret->str=0;\
ret->ivalue=*pival;\
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
pstring = ret->str;\
RETURN
#define READ_OP_INT_STRING_TIM(op,pival,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
*ret->str=0;\
ret->ivalue=*pival;\
accessor->send(&message,timeout);\
*pival = ret->ivalue;\
pstring = ret->str;\
RETURN
/***************************/

#define WRITE_OP_AX_STRING_TIM(op,ax,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
strncpy(idata->str,(char*)pstring,MAX_STR_SIZE); \
accessor->send(&message,timeout);\
RETURN

#define WRITE_OP_STRING_TIM(op,pstring,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
if(pstring) {strncpy(idata->str,(char*)pstring,MAX_STR_SIZE);}else {*idata->str=0;} \
SEND_AND_RETURN_TIM(timeout)

/***************************/


#define WRITE_OP_STRING_STRING_TIM(op,pstring,pstring2,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
if(pstring) strncpy(idata->str,(char*)pstring,MAX_STR_SIZE); \
if(pstring2) strncpy(idata->str2,(char*)pstring2,MAX_STR_SIZE); \
SEND_AND_RETURN

#define WRITE_OP_AX_STRING_STRING_TIM(op,ax,pstring,pstring2,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
if(pstring) strncpy(idata->str,(char*)pstring,MAX_STR_SIZE); \
if(pstring2) strncpy(idata->str2,(char*)pstring2,MAX_STR_SIZE); \
idata->axis=ax;\
SEND_AND_RETURN

/***************************/
#define READ_OP_AX_STRING_RETSTRING_TIM(op,ax,pstring,pstring2,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
if(pstring.size()) strncpy(idata->str,pstring.c_str(),MAX_STR_SIZE); \
idata->axis=ax;\
accessor->send(&message,timeout);\
pstring2=ret->str; \
RETURN
/***************************/
/* 
#define READ_OP_INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message);\
*pival = ret->ivalue;\
return ret->result;
*/

/***************************/
#define READ_OP_64INT_TIM(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;\
RETURN

#define READ_OP_AX_64INT_TIM(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
ret->alarm_mask=0;\
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;\
RETURN
/***************************/


#define READ_OP_64INT_TIM_NORET(op,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;

#define READ_OP_AX_64INT_TIM_NORET(op,ax,pival,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
accessor->send(&message,timeout);\
*pival = ret->alarm_mask;\
RETURN
/***************************/

#define READ_OP_2FLOAT_TIM(op,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
accessor->send(&message,timeout);\
*pfval0 = ret->fvalue0;\
*pfval1 = ret->fvalue1;\
RETURN

#define READ_OP_AX_2FLOAT_TIM(op,ax,pfval0,pfval1,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
accessor->send(&message,timeout);\
*pfval0 = ret->fvalue0;\
*pfval1 = ret->fvalue1;\
RETURN
/***************************/

#define WRITE_OP_TIM(op,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
SEND_AND_RETURN

#define WRITE_OP_AX_TIM(op,ax,timeout) \
PREPARE_OP_RET_INT_TIMEOUT(op,timeout); \
idata->axis=ax;\
SEND_AND_RETURN

/***************************/
#define ACT_STD_TIMEOUT 4999
uint64_t GeneralTimeout = ACT_STD_TIMEOUT;
int ChaosActuatorInterface::initACT(int axis,void*d){
    return impl->initACT(axis,d);
   // WRITE_OP_STRING_TIM(OP_INIT,(char*)d, GeneralTimeout);
}
int ChaosActuatorInterface::configAxis(int axis,void*d){
    //WRITE_OP_STRING_TIM(OP_CONFIGAXIS,(char*)d,60000);
    return impl->configAxis(axis,d);
}

int ChaosActuatorInterface::deinitACT(int32_t axisID){
    //WRITE_OP_AX_TIM(OP_DEINIT,axisID, GeneralTimeout);
    return impl->deinitACT(axisID);

}

uint64_t ChaosActuatorInterface::setGeneralInterfaceTimeout(uint64_t timeo_ms)
{

	GeneralTimeout = timeo_ms;
	return GeneralTimeout;
}
int ChaosActuatorInterface::setTimeout(int32_t axisID,uint64_t timeo_ms) {
 //   WRITE_OP_AX_TIM(OP_SET_TIMEOUT,axisID,timeo_ms);
 return impl->setTimeout(axisID,timeo_ms);
    
}
uint64_t ChaosActuatorInterface::getGeneralInterfaceTimeout()
{
	return  GeneralTimeout;
}
int ChaosActuatorInterface::getTimeout(int32_t axisID,uint64_t* timeo_ms) {
    //READ_OP_AX_64INT_TIM(OP_GET_TIMEOUT,axisID,timeo_ms, GeneralTimeout);
    return impl->getTimeout(axisID,timeo_ms);
}

int ChaosActuatorInterface::getPosition(int32_t axisID,::common::actuators::AbstractActuator::readingTypes readingType,double *deltaPosition_mm) {
  //  READ_OP_AX_FLOAT_PARAM_INT(OP_GET_POSITION,axisID,readingType,deltaPosition_mm, GeneralTimeout);
  return impl->getPosition(axisID,readingType,deltaPosition_mm);
}

int ChaosActuatorInterface::resetAlarms(int32_t axisID,uint64_t alrm){
    
   // WRITE_OP_AX_64INT_TIM(OP_RESET_ALARMS,axisID,alrm, GeneralTimeout);
   return impl->resetAlarms(axisID,alrm);
}

int ChaosActuatorInterface::getAlarms(int32_t axisID,uint64_t*alrm,std::string& desc){
    //READ_OP_AX_INT64_STRING_TIM(OP_GET_ALARMS,axisID,alrm,desc, GeneralTimeout);
    return impl->getAlarms(axisID,alrm,desc);
}

int ChaosActuatorInterface::moveRelative(int32_t axisID,double mm) {
   // float param=(float) mm;
  //  WRITE_OP_AX_FLOAT_TIM(OP_MOVE_RELATIVE_MM,axisID,param, GeneralTimeout);
  return impl->moveRelative(axisID,mm);
    
}
int ChaosActuatorInterface::moveAbsolute(int32_t axisID,double mm) {
   // float param=(float) mm;
    //WRITE_OP_AX_FLOAT_TIM(OP_MOVE_ABSOLUTE_MM,axisID,param, GeneralTimeout);
    return impl->moveAbsolute(axisID,mm);
    
}

int ChaosActuatorInterface::stopMotion(int32_t axisID){
    //DPRINT("ALEDEBUG STOP MOTION IN INTERFACE\n");
   // WRITE_OP_AX_TIM(OP_STOP_MOTION,axisID, GeneralTimeout);
   return impl->stopMotion(axisID);
}
int ChaosActuatorInterface::hardreset(int32_t axisID,bool mode){
   // WRITE_OP_AX_INT_TIM(OP_HARD_RESET,axisID,mode, GeneralTimeout);
   return impl->hardreset(axisID,mode);
}

int ChaosActuatorInterface::soft_homing(int32_t axisID,double positionToSet){
   // WRITE_OP_AX_INT_TIM(OP_HOMING,axisID,mode, GeneralTimeout);
   return impl->soft_homing(axisID,positionToSet);
}


int ChaosActuatorInterface::homing(int32_t axisID,homingType mode){
   // WRITE_OP_AX_INT_TIM(OP_HOMING,axisID,mode, GeneralTimeout);
   return impl->homing(axisID,mode);
}


int ChaosActuatorInterface::poweron(int32_t axisID,int on){
    
   // WRITE_OP_AX_INT_TIM(OP_POWERON,axisID,on, GeneralTimeout);
   return impl->poweron(axisID,on);

}

int ChaosActuatorInterface::getState(int32_t axisID,int* state,std::string& desc){
   // READ_OP_AX_INT_STRING_TIM(OP_GET_STATE,axisID, state, desc, GeneralTimeout);
    //WRITE_OP_TIM(OP_GET_STATE,0);
    return impl->getState(axisID,state,desc);
}


int ChaosActuatorInterface::listParameters(std::string& dataset){
    int state=0;
  //  READ_OP_INT_STRING_TIM(OP_LISTPARAMETERS, &state, dataset, GeneralTimeout);
  return impl->listParameters(dataset);
}

int ChaosActuatorInterface::getSWVersion(int32_t axisID,std::string& ver){
    //int state=0;
   // READ_OP_AX_INT_STRING_TIM(OP_GET_SWVERSION,axisID, &state, ver, GeneralTimeout);
   return impl->getSWVersion(axisID,ver);
}

int ChaosActuatorInterface::getHWVersion(int32_t axisID,std::string&ver){
  //  int state=0;
  //  READ_OP_AX_INT_STRING_TIM(OP_GET_HWVERSION,axisID, &state, ver, GeneralTimeout);
  return impl->getHWVersion(axisID,ver);

}


uint64_t ChaosActuatorInterface::getFeatures() {
   // uint64_t feats=0;
   // READ_OP_64INT_TIM_NORET(OP_GET_FEATURE,&feats, GeneralTimeout);
   // return feats;
   return impl->getFeatures();
}
int ChaosActuatorInterface::setParameter(int32_t axisID,const std::string parName,const std::string value) {
	//WRITE_OP_AX_STRING_STRING_TIM(OP_SETPARAMETER,axisID,parName.c_str(),value.c_str(), GeneralTimeout);
    return impl->setParameter(axisID,parName,value);
}


int ChaosActuatorInterface::getParameter(int axisID,std::string parName,std::string& resultString) {
	//READ_OP_AX_STRING_RETSTRING_TIM(OP_GETPARAMETER,axisID,parName,resultString, GeneralTimeout);
    return impl->getParameter(axisID,parName,resultString);
}






