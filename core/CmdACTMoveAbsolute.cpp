/*
 *	CmdACTMoveAbsolute.cpp
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
#include "CmdACTMoveAbsolute.h"

#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

#define SCLAPP_ INFO_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTMoveAbsolute) << "[" << getDeviceID() << "] "


namespace own =  driver::actuator;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;


BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTMoveAbsolute,CMD_ACT_MOVE_ABSOLUTE_ALIAS,
                                                          "Move from current position to an absolute position (mm)",
                                                          "0cf52f76-55eb-4rt3-8712-3d54484043d8")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_MM_OFFSET, "position mm",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

//// return the implemented handler           //************************** commentato *****************************
//uint8_t own::CmdACTMoveAbsolute::implementedHandler(){
//    return	AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}
//uint64_t computed_timeout;

void own::CmdACTMoveAbsolute::setHandler(c_data::CDataWrapper *data) {
//    chaos::common::data::RangeValueInfo position_sp_attr_info;
//    chaos::common::data::RangeValueInfo attributeInfo;
    AbstractActuatorCommand::setHandler(data);
    
    double max_position=0,min_position=0;
    int err = 0;

    float positionToReach = 0.f;
    chaos::common::data::RangeValueInfo attr_info;
    
    double currentPosition;
    
//    axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");
//    o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");
//    o_position_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp");
//    i_speed = (double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");		
//    i_command_timeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "command_timeout");
//    __i_delta_setpoint = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__delta_setpoint");
//    __i_setpoint_affinity = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__setpoint_affinity");
//    tmpInt = (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ;
//    readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;
         
    getDeviceDatabase()->getAttributeRangeValueInfo("position_sp", attr_info);
    setAlarmSeverity("position_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    setAlarmSeverity("position_out_of_set", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    
    // REQUIRE MIN MAX SET IN THE MDS
    if (attr_info.maxRange.size()) {
        max_position = atof(attr_info.maxRange.c_str());
        SCLDBG_ << "max_position max=" << max_position;

    } else {
        SCLERR_ << "Not defined maximum 'position_sp' attribute, quitting command";
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined maximum 'position_sp' attribute, quitting command" );
        setAlarmSeverity("position_sp_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        BC_FAULT_RUNNING_PROPERTY;// ********** aggiunto **************
        return;
    }

    // REQUIRE MIN MAX POSITION IN THE MDS
    if (attr_info.minRange.size()) {
        min_position = atof(attr_info.minRange.c_str());
        SCLDBG_ << "min_position min=" << min_position;
    } else {
        SCLERR_ << "not defined minimum 'position_sp' attribute, quitting command";
        setAlarmSeverity("position_sp_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined minimum 'position_sp' attribute, quitting command" );     
        BC_FAULT_RUNNING_PROPERTY;
        return;
    }
    
    // ********************* a cosa servono **********************
    SCLDBG_<<"minimum working value:"<<*p_minimumWorkingValue;
    SCLDBG_<<"maximum, working value:"<<*p_maximumWorkingValue;
    
    SCLDBG_ << "check data";
    
    if(!data ||
	!data->hasKey(CMD_ACT_MM_OFFSET)) {
        SCLERR_ << "Position millimeters parameter not present";
        setAlarmSeverity("Offset_mm_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        BC_FAULT_RUNNING_PROPERTY;
	return;
    }
    if(!data->isDoubleValue(CMD_ACT_MM_OFFSET)) {
	SCLERR_ << "Position millimeters parameter is not a Double data type";
        setAlarmSeverity("Offset_mm_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
	BC_FAULT_RUNNING_PROPERTY;
	return;
    }
        
    positionToReach = static_cast<float>(data->getDoubleValue(CMD_ACT_MM_OFFSET));
    if(isnan(positionToReach)==true){
        SCLERR_ << "Position parameter is not a valid double number (nan?)";
        setAlarmSeverity("Offset_mm_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        BC_FAULT_RUNNING_PROPERTY;
        return;
    }
    
    // Controllo setpoint finale: se tale valore appartiene al range [min_position-tolmin,max_position+tolmax]
    double tolmax = std::abs(max_position*0.3);
    double tolmin = std::abs(min_position*0.3);
    
    if (((positionToReach) > (max_position+tolmax)) || ((positionToReach)< (min_position-tolmin)))
    {
        setAlarmSeverity("final setpoint_mm_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
        SCLERR_ << "Finale position "<<positionToReach<< " out of range ( " << min_position << ","<< max_position <<") the command won't be executed";
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Final set point %1% outside the maximum/minimum 'position_sp' = tolerance \"max_position\":%2% \"min_position\":%3%" , % positionToReach % max_position % min_position));
        BC_FAULT_RUNNING_PROPERTY;
        return;
    }
        
    // Ma lo spostamento da effettuare e' maggiore dello spostamento minimo *p_resolution?
    AbstractActuatorCommand::acquireHandler(); // Per aggiornare al momento piu opportuno *o_position    
    currentPosition=*o_position;    
    double deltaPosition = std::abs(positionToReach-currentPosition);
    if(deltaPosition<*p_resolution){
        SCLDBG_ << "operation inibited because of resolution:" << *p_resolution;
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("operation inibited because of resolution %1% , position set point %2%",%*p_resolution %positionToReach ));
        *i_position=positionToReach;
        getAttributeCache()->setInputDomainAsChanged();
        BC_END_RUNNING_PROPERTY;
        return;
    }    

    SCLDBG_ << "compute timeout for moving Absolute = " << positionToReach;
	
    //numero di secondi, dopo lo moltiplichiamo per 1 milione (volendo da micro)
    uint64_t computed_timeout; // timeout will be expressed in [ms]
    if (*i_speed!= 0)
    {
        computed_timeout  = uint64_t((deltaPosition / *i_speed)*1000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS; 
        computed_timeout = std::max(computed_timeout,(uint64_t)*p_setTimeout);
    
    }   else computed_timeout=(uint64_t)*p_setTimeout;
    
    //setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout)
    SCLDBG_ << "Calculated timeout is = " << computed_timeout;
    setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);
    
    //slow_acquisition_index = false;
    *i_position=positionToReach;
    setWorkState(true);
    getAttributeCache()->setInputDomainAsChanged();
    setAlarmSeverity("position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelClear);
    
    SCLDBG_ << "o_position_sp is = " << *i_position;
    
    if(*o_stby){
        // we are in standby only the SP is set
        SCLDBG_ << "we are in standby we cannot start moving to: "<<*i_position;
        setWorkState(false);
        BC_END_RUNNING_PROPERTY;
        return;
    } 
    
    SCLDBG_ << "Move to position " << positionToReach << "reading type " << readTyp;
    
    if((err = actuator_drv->moveAbsoluteMillimeters(*axID,positionToReach)) != 0) {
        SCLERR_<<"## error setting moving absolute of "<<positionToReach;
        setAlarmSeverity("position_invalid_set", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
        setWorkState(false);
        BC_FAULT_RUNNING_PROPERTY;
        return;
    }
	
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command move absolute :%1% timeout %2%") % positionToReach % computed_timeout) );
    BC_EXEC_RUNNING_PROPERTY;
}

void own::CmdACTMoveAbsolute::acquireHandler() {   //************ modificato in maniera analoga al file CmdACTMoveRelative.cpp **********
	//acquire the current readout
        AbstractActuatorCommand::acquireHandler();
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdACTMoveAbsolute::ccHandler() {
    //check if we are in the delta of the setpoint to end the command
    double delta_position_reached = std::abs(*i_position - *o_position);
    //SCLDBG_ << "ccH MoveABsolute Readout: "<< *o_position <<" SetPoint: "<< *o_position_sp <<" Delta to reach: " << delta_position_reached << " computed Timeout " << computed_timeout ;
    SCLDBG_ << "Readout: "<< *o_position <<" SetPoint: "<< *i_position <<" Delta to reach: " << delta_position_reached;
    if(delta_position_reached <= *p_resolution || delta_position_reached<*p_warningThreshold) 
    {
	uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	//the command is endedn because we have reached the affinitut delta set
	SCLDBG_ << "[metric ]Set point reached with - delta: "<< delta_position_reached <<" sp: "<< *i_position <<" affinity check " << *p_resolution <<  " warning threshold: " << *p_warningThreshold << " mm in " << elapsed_msec << " milliseconds";
	setWorkState(false);
        BC_END_RUNNING_PROPERTY;
    }
    if(*o_alarms) {
        SCLERR_ << "We got alarms on actuator/slit so we end the command";
        setWorkState(false);
	BC_END_RUNNING_PROPERTY;
    }
}

bool own::CmdACTMoveAbsolute::timeoutHandler() {
    uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
    double delta_position_reached = std::abs(*i_position - *o_position);
    
    metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("timeout, delta position remaining %1%",%delta_position_reached));
    
    SCLDBG_ << "  TIM MoveABsolute Readout: "<< *o_position <<" SetPoint: "<< *i_position<<" Delta to reach: " << delta_position_reached;
    SCLDBG_ << "  TIM MoveABsolute  resolution: " << *p_resolution;
    if(delta_position_reached <= *p_resolution || delta_position_reached <*p_warningThreshold) {
        SCLDBG_ << "[metric] Setpoint reached on timeout with set point " << *i_position<< " readout position" << *o_position << " resolution" << *p_resolution << " warning threshold " << *p_warningThreshold << " in " << elapsed_msec << " milliseconds";
	
    }else {

        SCLERR_ << "[metric] Setpoint not reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
        setAlarmSeverity("value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
    }
    setWorkState(false);
    BC_END_RUNNING_PROPERTY;
    return false;
}
