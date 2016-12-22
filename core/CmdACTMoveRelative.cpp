/*
 *	CmdACTMoveRelative.cpp
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


#include "CmdACTMoveRelative.h"

#include <cmath>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

#define SCLAPP_ INFO_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLDBG_ DBG_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "
#define SCLERR_ ERR_LOG(CmdACTMoveRelative) << "[" << getDeviceID() << "] "


namespace own =  driver::actuator;
namespace c_data = chaos::common::data;
namespace chaos_batch = chaos::common::batch_command;


BATCH_COMMAND_OPEN_DESCRIPTION_ALIAS(driver::actuator::,CmdACTMoveRelative,CMD_ACT_MOVE_RELATIVE_ALIAS,
                                                          "Move from current position to an offset (mm)",
                                                          "0cf52f73-55eb-4e13-8712-3d54486040d8")
BATCH_COMMAND_ADD_DOUBLE_PARAM(CMD_ACT_MM_OFFSET, "offset in mm",chaos::common::batch_command::BatchCommandAndParameterDescriptionkey::BC_PARAMETER_FLAG_MANDATORY)
BATCH_COMMAND_CLOSE_DESCRIPTION()

// return the implemented handler    //************************** commentato *****************************
//uint8_t own::CmdACTMoveRelative::implementedHandler(){
//    return	AbstractActuatorCommand::implementedHandler()|chaos_batch::HandlerType::HT_Acquisition;
//}

void own::CmdACTMoveRelative::setHandler(c_data::CDataWrapper *data) {
        //chaos::common::data::RangeValueInfo position_sp_attr_info;   // ************* Commentato *************
        //chaos::common::data::RangeValueInfo attributeInfo;           // ************* Commentato *************
	AbstractActuatorCommand::setHandler(data);
        // ********** aggiunto ************** Ha aggiornato anche position....

	double max_position=0,min_position=0;
	int err = 0;
	//int state;
        //int *tmpInt;            // ************* Commentato *************
        double currentPosition;
	//std::string state_str;
	float offset_mm = 0.f;
	chaos::common::data::RangeValueInfo attr_info;
	//o_position = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position");     // ************* Commentato *************
//	o_position_sp = getAttributeCache()->getRWPtr<double>(DOMAIN_OUTPUT, "position_sp"); // ************* Commentato *************
//	i_speed = ( double*) getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "speed");   // ************* Commentato *************		
//	i_command_timeout = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "command_timeout");  // ************* Commentato ************* 
//	__i_delta_setpoint = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__delta_setpoint");  // ************* Commentato *************
//	__i_setpoint_affinity = getAttributeCache()->getROPtr<double>(DOMAIN_INPUT, "__setpoint_affinity");  // ************* Commentato *************
	//axID = getAttributeCache()->getROPtr<uint32_t>(DOMAIN_INPUT, "axisID");    // ************* Commentato *************
//        tmpInt =  (int*) getAttributeCache()->getROPtr<int32_t>(DOMAIN_INPUT, "readingType") ; // ************* Commentato *************  
//        readTyp=(::common::actuators::AbstractActuator::readingTypes) *tmpInt;     // ************* Commentato *************
        getDeviceDatabase()->getAttributeRangeValueInfo("position", attr_info);
        setAlarmSeverity("command_error", chaos::common::alarm::MultiSeverityAlarmLevelClear);// ********** aggiunto **************
          
        // REQUIRE MIN MAX SET IN THE MDS

        if (attr_info.maxRange.size()) {
            max_position = atof(attr_info.maxRange.c_str());
            SCLDBG_ << "max_position max=" << max_position;

        } else {
            SCLERR_ << "not defined maximum 'position_sp' attribute, quitting command";
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined maximum 'position_sp' attribute, quitting command" ); 
            BC_FAULT_RUNNING_PROPERTY;// ********** aggiunto **************
            return;
        }

        // REQUIRE MIN MAX POSITION IN THE MDS
        if (attr_info.minRange.size()) {
            min_position = atof(attr_info.minRange.c_str());

            SCLDBG_ << "min_position min=" << min_position;
        } else {
            SCLERR_ << "not defined minimum 'position_sp' attribute, quitting command";
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,"not defined minimum 'position_sp' attribute, quitting command" );     
                  
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }

        // ********************* a cosa servono **********************
        //SCLDBG_<<"minimum working value:"<<*p_minimumWorkingValue;
        //SCLDBG_<<"maximum, working value:"<<*p_maximumWorkingValue;
        
        //set comamnd timeout for this instance
        //SCLDBG_ << "Checking for timeout";
	
	SCLDBG_ << "check data";
	if(!data ||
	   !data->hasKey(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter not present";
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
	if(!data->isDoubleValue(CMD_ACT_MM_OFFSET)) {
		SCLERR_ << "Offset millimeters parameter is not a Double data type";
		BC_FAULT_RUNNING_PROPERTY;
		return;
	}
        
        offset_mm = 0;
        offset_mm = static_cast<float>(data->getDoubleValue(CMD_ACT_MM_OFFSET));
        //SCLAPP_<<"offset_mm:"<<offset_mm;
        
        if(isnan(offset_mm)==true){
            SCLERR_ << "Offset millimeters parameter is not a valid double number (nan?)";
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }

        // Controllo setpoint finale: se tale valore appartiene al range [min_position-tolmin,max_position+tolmax]
        double tolmax = std::abs(max_position*0.3);
        double tolmin = std::abs(min_position*0.3);
        double position;
        if ((err = actuator_drv->getPosition(*axID,readTyp,&position))==0) {
            //LOG_AND_TROW(SCLERR_, 1, boost::str(boost::format("Error fetching position with code %1%") % err));
            *o_position = position;
        } else {
            //*o_position = position;
            SCLERR_ <<boost::str( boost::format("Error calling driver on get Position readout with code %1%") % err);
        }
        currentPosition=*o_position;
        double newPosition=currentPosition+offset_mm;
        if((newPosition) > (max_position+tolmax)|| (newPosition)< (min_position-tolmin)){ // nota: *o_position aggiornata inizialmente da AbstractActuatorCommand::acquireHandler();  
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelError,CHAOS_FORMAT("Final set point %1% outside the maximum/minimum 'position_sp' = tolerance \"max_position\":%2% \"min_position\":%3%" , % (currentPosition + offset_mm) % max_position % min_position));
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        // Ma lo spostamento da effettuare e' maggiore dello spostamento minimo *p_resolution?
        // ****************** Nota: *p_resolution sostituisce il vecchio *__i_delta_setpoint *********************
        if(std::abs(offset_mm)<*p_resolution){
            SCLDBG_ << "operation inibited because of resolution:" << *p_resolution;
            metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("operation inibited because of resolution %1% , delta position %2%",%*p_resolution %offset_mm ));
            *i_position=newPosition;
            getAttributeCache()->setInputDomainAsChanged();
            BC_END_RUNNING_PROPERTY;
            return;
        }
        
        SCLDBG_ << "Compute timeout for moving relative = " << offset_mm;
        
        uint64_t computed_timeout; // timeout will be expressed in [ms]
	if (*i_speed != 0)
        {
            computed_timeout  = uint64_t((offset_mm / *i_speed)*1000000000) + DEFAULT_MOVE_TIMETOL_OFFSET_MS; 
                                                     //i_speed is expressed in [mm/s]
            computed_timeout=std::max(computed_timeout,(uint64_t)*p_setTimeout);
        }
        else computed_timeout=(uint64_t)*p_setTimeout;
	
	SCLDBG_ << "Calculated timeout is = " << computed_timeout;
	setFeatures(chaos_batch::features::FeaturesFlagTypes::FF_SET_COMMAND_TIMEOUT, computed_timeout);
	
//	if(*__i_setpoint_affinity) {
//		affinity_set_delta = *__i_setpoint_affinity;
//	} else {
//		affinity_set_delta = 1;
//	}
//	SCLDBG_ << "The setpoint affinity value is of +-" << affinity_set_delta << " of millimeters";
        // ************* Nota: affinity_set_delta veniva gestito nelle funzioni cchandler() e nel timeouthandler() ***************** 
        

	//assign new position setpoint
	//slow_acquisition_index = false;
        *i_position=newPosition; //**************** nota: *i_position rimpiazza *o_position_sp *************************
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

        SCLDBG_ << "Move to position " << newPosition << "reading type " << readTyp;
        
	if((err = actuator_drv->moveRelativeMillimeters(*axID,offset_mm)) != 0) {
            SCLERR_<<"## error setting moving relative of "<<offset_mm;
            setAlarmSeverity("command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            setWorkState(false);
            BC_FAULT_RUNNING_PROPERTY;
            return;
        }
        
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelInfo,boost::str( boost::format("performing command move relative :%1% timeout %2%") % offset_mm % computed_timeout) );
        //************* actuator_drv->accessor->base_opcode_priority=100; ********************* commentato
	BC_NORMAL_RUNNING_PROPERTY;
}

void own::CmdACTMoveRelative::acquireHandler() {          //OK
    
        //acquire the current readout
        AbstractActuatorCommand::acquireHandler();
	//force output dataset as changed
	getAttributeCache()->setOutputDomainAsChanged();
}

void own::CmdACTMoveRelative::ccHandler() {
	//check if we are in the delta of the setpoint to end the command
	double delta_position_reached = std::abs(*o_position-*i_position);
	SCLDBG_ << "Readout: "<< *o_position <<" SetPoint: "<< *i_position <<" Delta to reach: " << delta_position_reached;
 	if (((*o_status_id) & ::common::actuators::ACTUATOR_INMOTION)==0)
        {
		setWorkState(false);
                BC_END_RUNNING_PROPERTY;	
        }
 	if (((*o_status_id) & ::common::actuators::ACTUATOR_POWER_SUPPLIED)==0)
        {
		int err;
		if (err=actuator_drv->stopMotion(*axID)!= 0)
		{
		     SCLERR_<<"## error while stopping motion";
            		setAlarmSeverity("command_error", chaos::common::alarm::MultiSeverityAlarmLevelHigh);
            		setWorkState(false);
            		BC_FAULT_RUNNING_PROPERTY;
		}
		setWorkState(false);
                BC_END_RUNNING_PROPERTY;	
	   
        }      

	if(delta_position_reached <= *p_resolution) 
        {
		uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		//the command is endedn because we have reached the affinitut delta set
		SCLDBG_ << "[metric ]Set point reached with - delta: "<< delta_position_reached <<" sp: "<< *i_position <<" affinity check " << *p_resolution << " mm in " << elapsed_msec << " milliseconds";
		setWorkState(false);
                BC_END_RUNNING_PROPERTY;	
        }
        
	if(*o_alarms) {
		SCLERR_ << "We got alarms on actuator/slit so we end the command";
                setWorkState(false); 
		BC_END_RUNNING_PROPERTY;  
	}
}

bool own::CmdACTMoveRelative::timeoutHandler() {
    
        uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	double delta_position_reached = std::abs(*i_position - *o_position);
	//uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
	//move the state machine on fault
	//setWorkState(false);  ************* commentato *******************
        metadataLogging(chaos::common::metadata_logging::StandardLoggingChannel::LogLevelWarning,CHAOS_FORMAT("timeout, delta position remaining %1%",%delta_position_reached));
        
        
	if(delta_position_reached <= *p_resolution) {
		
		SCLDBG_ << "[metric] Setpoint reached on timeout with set point " << *i_position<< " readout position" << *o_position << " resolution" << *p_resolution << " warning threshold " << *p_warningThreshold << " in " << elapsed_msec << " milliseconds";
		//the command is endedn because we have reached the affinitut delta set
                
                BC_END_RUNNING_PROPERTY;
               
                
	}else {
                //uint64_t elapsed_msec = chaos::common::utility::TimingUtil::getTimeStamp() - getSetTime();
		SCLERR_ << "[metric] Setpoint not reached on timeout with readout position " << *o_position << " in " << elapsed_msec << " milliseconds";
		setAlarmSeverity("position_value_not_reached", chaos::common::alarm::MultiSeverityAlarmLevelWarning);
                //FIRE OUT OF SET
		//BC_END_RUNNING_PROPERTY; // ************* commentato *******************
		
                BC_FAULT_RUNNING_PROPERTY;
                
	}
        setWorkState(false);
        return false;
}
