/*
CmdACTHoming.cpp
!CHAOS
Created by CUGenerator

Copyright 2013 INFN, National Institute of Nuclear Physics
Licensed under the Apache License, Version 2.0 (the "License")
you may not use this file except in compliance with the License.
      You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/
#include "CmdACTHoming.h"

#include <cmath>
#include  <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#define SCLAPP_ INFO_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTHoming) << "[" << getDeviceID() << "] "
namespace own = driver::actuator;
namespace c_data =  chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;
BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTHoming,CMD_ACT_HOMING_ALIAS,
			"Calibrate the actuator reaching the home position",
			"f096d258-1690-11e6-8845-1f6ad6d4e676")
BATCH_COMMAND_ADD_INT32_PARAM(CMD_ACT_HOMINGTYPE,"homing Type",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()


// return the implemented handler
//uint8_t own::CmdACTHoming::implementedHandler(){
//  return      AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
// empty set handler
void own::CmdACTHoming::setHandler(c_data::CDataWrapper *data) 
{
        AbstractActuatorCommand::setHandler(data);
        
        int err = 0;
        //double max_homing_type=0,min_homing_type=0;
        double currentPosition;
        uint64_t computed_timeout;
        *p_stopCommandInExecution=false;
        chaos::common::data::RangeValueInfo attr_info;
        getDeviceDatabase()->getAttributeRangeValueInfo("homing_type", attr_info);
        setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelClear);
        //setAlarmSeverity("homing_type_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
        
        // REQUIRE MIN MAX SET IN THE MDS
//        if (attr_info.maxRange.size()) {
//            max_homing_type = atof(attr_info.maxRange.c_str());
//            SCLDBG_ << "max homing_type=" << max_homing_type;
//        } 
//        else {
//            SCLERR_ << "Not defined maximum 'homing_type' attribute, quitting command";
//            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined maximum 'homing_type' attribute, quitting command" );
//            setAlarmSeverity("homing_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
//            BC_FAULT_RUNNING_PROPERTY;// ********** aggiunto **************
//            return;
//        }
        
//        if (attr_info.minRange.size()) {
//            min_homing_type = atof(attr_info.minRange.c_str());
//            SCLDBG_ << "min homing_type=" << min_homing_type;
//        }
//        else {
//            SCLERR_ << "not defined minimum 'homing_type' attribute, quitting command";
//            setAlarmSeverity("homing_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
//            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined minimum 'position_sp' attribute, quitting command" );     
//            BC_FAULT_RUNNING_PROPERTY;
//            return;
//        }
        
        // Controllo natura dato in input
        SCLDBG_ << "check data";
        
        if(!data ||
            !data->hasKey(CMD_ACT_HOMINGTYPE)) {
            SCLERR_ << "Homing type parameter not present";
            //setAlarmSeverity("Homing_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        if(!data->isInt32Value(CMD_ACT_HOMINGTYPE)) {
            SCLERR_ << "Homing  parameter is not an integer data type";
            //setAlarmSeverity("Homing_type_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
    	int32_t homType = data->getInt32Value(CMD_ACT_HOMINGTYPE);
        if(isnan(homType)==true){
            SCLERR_ << "homType parameter is not a valid integer number (nan?)";
            //setAlarmSeverity("homType_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        // Controlliamo adesso se il dato appartiene all'input
//        if (((homType) > (max_homing_type)) || ((homType)< (min_homing_type)))
//        {
//            setAlarmSeverity("homing_type invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
//            SCLERR_ << "homing_type "<<homType<< " out of range ( " << min_homing_type << ","<< max_homing_type <<") the command won't be executed";
//            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("homing_type %1% outside the maximum/minimum 'homing_type' = tolerance \"max_homing_type\":%2% \"min_homing_type\":%3%" , % homType % max_homing_type % min_homing_type));
//            BC_FAULT_RUNNING_PROPERTY;
//            return;
//        }
        
        SCLDBG_ << "Compute timeout for homing operation of type = " << homType;
        //.......................
        AbstractActuatorCommand::acquireHandler(); // Per aggiornare al momento piu opportuno *o_position, *readTyp    
        currentPosition=* o_position;   
        
        if (*highspeed_homing!= 0)
        {
        computed_timeout  = uint64_t((std::abs(*o_position) / *highspeed_homing)*1000000000000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS; 
        computed_timeout = std::max(computed_timeout,(uint64_t)*p_setTimeout);
    
        }   else computed_timeout=(uint64_t)*p_setTimeout;
        
        setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout); 
        setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)100000); //*********** (uint64_t)100000 deve diventare un parametro ************
        
//        slow_acquisition_index = false;       *************** da inserire? **************
//        *i_position=positionToReach;
          setWorkState(true);                  // *************** da inserire? **************
//        getAttributeCache()->setInputDomainAsChanged();
//        setAlarmSeverity("position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);
        
//        if(*o_stby){
//            // we are in standby only the SP is set
//            SCLDBG_ << "we are in standby we cannot start homing operation of type: "<<homType;
//            setWorkState(false); 
//            BC_END_RUNNING_PROPERTY;
//            return;
//        } 
        
        SCLDBG_ << "Start homing operation of type " << homType; 
        if(*o_stby){
            // we are in standby only the SP is set
            SCLDBG_ << "we are in standby we cannot start homing operation: ";
            setWorkState(false);
            BC_END_RUNNING_PROPERTY;
            return;
        } 
        
        if(err = actuator_drv->homing(*axID,(::common::actuators::AbstractActuator::homingType) homType) < 0) 
        {
            SCLERR_<<"## error setting homing operation of type "<<homType;
            setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            setWorkState(false);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        homingTypeVar = homType;
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command homing of type:%1% timeout %2%") % homType % computed_timeout) );
        BC_EXEC_RUNNING_PROPERTY;
}

//  acquire handler
void own::CmdACTHoming::acquireHandler() {
    int err;
    //int state;
    //std::string state_str;
    //double position;
    SCLDBG_ << "Start Acquire Handler " ;
    
    //acquire the current readout
    AbstractActuatorCommand::acquireHandler(); // ********* Necessario per aggiornare solo stato e posizione in questo specifico caso ***************
    //force output dataset as changed
    getAttributeCache()->setOutputDomainAsChanged();
   
    SCLDBG_ << "Homing acquire before sending homing again";
    setWorkState(true);
    setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_SCHEDULER_DELAY, (uint64_t)1000000); //*********** (uint64_t)100000 deve diventare un parametro ************
    if((err = actuator_drv->homing(*axID,(::common::actuators::AbstractActuator::homingType) homingTypeVar)) < 0)
    {
        SCLERR_<<"## error setting homing operation of type "<<homingTypeVar;
        setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        setWorkState(false);
        BC_FAULT_RUNNING_PROPERTY;
        return;
    }
    homResult=err;
    SCLDBG_ <<" HOMERESULT " << homResult ;
}

//  correlation handler
void own::CmdACTHoming::ccHandler() {
	
    SCLDBG_ <<" CC Handler homResults " << homResult ;
    if (homResult == 0)
    {
        uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
        SCLDBG_ << "Homing operation completed in "<< elapsed_msec <<" milliseconds";
	setWorkState(false);
        BC_END_RUNNING_PROPERTY;
    }
    
 if (*p_stopCommandInExecution) // questa funzione dovrebbe essere considerata solo se e' in esecuzione il comando di stop
                                                                   // Il comando di stop potrebbe settare un membro della classe astratta.
    {
        *p_stopCommandInExecution=false;
        setWorkState(false);
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,("Error Homing not completed" ));
        setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        SCLDBG_ << "Exit from homing because of actutator is not in motion";
        BC_FAULT_RUNNING_PROPERTY;
    }
if (((*o_status_id) & ::common::actuators::ACTUATOR_POWER_SUPPLIED)==0)
        {
                int err;
                if (err=actuator_drv->stopMotion(*axID)!= 0)
                {
                     SCLERR_<<"## error while stopping motion";
                setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
                        setWorkState(false);
                        BC_FAULT_RUNNING_PROPERTY;
                        return;

                }
                setWorkState(false);
                BC_END_RUNNING_PROPERTY;

        }


//    if(*o_alarms) {
//        SCLERR_ << "We got alarms on actuator/slit so we end the command";
//        setWorkState(false);
//	BC_END_RUNNING_PROPERTY;
//    }
}

// empty timeout handler
bool own::CmdACTHoming::timeoutHandler() {
    
    uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("timeout, position remaining %1%, elapsed time %2%" , % * o_position % elapsed_msec));
    // E' necessario, per come è implmentata la procedura di homing,
    // inviare un comando di stop di movimentazione.
    int err;
    if((err = actuator_drv->stopMotion(*axID)) != 0) {
        SCLERR_<<"## time out ";
        setAlarmSeverity("homing operation failed", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        setWorkState(false);
        BC_FAULT_RUNNING_PROPERTY;
        return false;
    }
    
    setWorkState(false);
    BC_END_RUNNING_PROPERTY;
    return false;
}
