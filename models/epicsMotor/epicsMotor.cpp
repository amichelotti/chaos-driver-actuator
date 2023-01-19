/*



 *
 *	!CHAOS
 *	Created by Andrea Michelotti
 *
 *    	Copyright 2022 INFN, National Institute of Nuclear Physics
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
#include "epicsMotor.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <common/actuators/core/AbstractActuator.h>
#include <driver/epics/driver/EpicsPVAccessDriver.h>
/**
https://epics.anl.gov/bcda/synApps/motor/motorRecord.html

Alphabetical list of record-specific fields
NOTE: Hot links in this table take you only to the section in which the linked item is described in detail. You'll probably have to scroll down to find the actual item.
Name	Access	Prompt	Data type	Comment
ACCL	R/W	Seconds to Velocity	DOUBLE	acceleration time
ADEL	R/W	Archive Deadband	DOUBLE	Deadband on RBV archive monitor
ALST	R	Last Valve Archived	DOUBLE	Last RBV value to be archived
ATHM	R	At HOME	SHORT	uses the HOME switch
BACC	R/W	BL Seconds to Veloc.	DOUBLE	backlash acceleration time
BDST	R/W	BL Distance (EGU)	DOUBLE	backlash distance
BVEL	R/W	BL Velocity (EGU/s)	DOUBLE	backlash speed
CARD	R	Card Number	SHORT	EPICS card #
CBAK	None	Callback structure	NOACCESS	
CDIR	R	Raw commanded direction	SHORT	(1:"Pos", 0:"Neg")
CNEN	R/W	Enable control	RECCHOICE	(0:"Disable", 1:"Enable")
DCOF	R/W	Derivative Gain	DOUBLE	
DHLM	R/W*	Dial High Limit	DOUBLE	
DIFF	R	Difference dval-drbv	DOUBLE	
DINP	R/W	DMOV Input Link	INLINK	
DIR	R/W*	User Direction	RECCHOICE	(0:"Pos", 1:"Neg")
DLLM	R/W*	Dial Low Limit	DOUBLE	
DLY	R/W	Readback settle time (s)	DOUBLE	
DMOV	R	Done moving to value	SHORT	The "done" flag
DOL	R	Desired Output Loc	INLINK	only for closed-loop mode
DRBV	R	Dial Readback Value	DOUBLE	
DVAL	R/W*	Dial Desired Value	DOUBLE	
EGU	R/W	Engineering Units	STRING	
ERES	R/W*	Encoder Step Size (EGU)	DOUBLE	
FOF	R/W	Freeze Offset	SHORT	
FOFF	R/W	Offset-Freeze Switch	RECCHOICE	(0:"Variable", 1:"Frozen")
FRAC	R/W	Move Fraction	DOUBLE	
HHSV	R/W*	Hihi Severity	GBLCHOICE	
HIGH	R/W*	High Alarm Limit	DOUBLE	
HIHI	R/W*	Hihi Alarm Limit	DOUBLE	
HLM	R/W*	User High Limit	DOUBLE	
HLS	R	At High Limit Switch	SHORT	
HLSV	R/W*	HW Lim. Violation Svr	GBLCHOICE	
HOMF	R/W*	Home Forward	SHORT	
HOMR	R/W*	Home Reverse	SHORT	
HOPR	R/W	High Operating Range	DOUBLE	
HSV	R/W*	High Severity	GBLCHOICE	
HVEL	R/W*	Home Velocity	DOUBLE	
ICOF	R/W	Integral Gain	DOUBLE	
IGSET	R/W	Ignore SET Field	SHORT	(0:Normal operation, 1: SET is ignored)
INIT	R/W	Startup commands	STRING	
JAR	R/W	Jog Acceleration (EGU/s^2)	DOUBLE	
JOGF	R/W*	Jog motor Forward	SHORT	careful!
JOGR	R/W*	Jog motor Reverse	SHORT	careful!
JVEL	R/W	Jog Velocity	DOUBLE	
LDVL	R	Last Dial Des Val	DOUBLE	
LLM	R/W*	User Low Limit	DOUBLE	
LLS	R	At Low Limit Switch	SHORT	
LLSV	R/W*	Lolo Severity	GBLCHOICE	
LOCK	R/W*	Soft Channel Position Lock	RECCHOICE	(0:"NO", 1:"YES")
LOLO	R/W*	Lolo Alarm Limit	DOUBLE	
LOPR	R/W	Low Operating Range	DOUBLE	
LOW	R/W*	Low Alarm Limit	DOUBLE	
LRLV	R	Last Rel Value	DOUBLE	
LRVL	R	Last Raw Des Val	DOUBLE	
LSPG	R	Last SPMG	RECCHOICE	(See SPMG)
LSV	R/W*	Low Severity	GBLCHOICE	
LVAL	R	Last User Des Val	DOUBLE	
LVIO	R	Limit violation	SHORT	
MDEL	R/W	Monitor Deadband	DOUBLE	Deadband on RBV valve monitor
MLST	R	Last Valve Monitored	DOUBLE	Last RBV value to be value monitored
MIP	R	Motion In Progress	SHORT	
MISS	R	Ran out of retries	SHORT	
MMAP	R	Monitor Mask	ULONG	
MOVN	R	Motor is moving	SHORT	Don't confuse with DMOV
MRES	R/W*	Motor Step Size (EGU)	DOUBLE	
MSTA	R	Motor Status	ULONG	
NMAP	R	Monitor Mask	ULONG	
NTM	R/W*	New Target Monitor	RECCHOICE	(0:"NO", 1:"YES")
NTMF	R/W*	New Target Monitor Deadband Factor	SHORT	Determines NTM deadband; NTMF >= 2
OFF	R/W	User Offset (EGU)	DOUBLE	
OMSL	R/W	Output Mode Select	GBLCHOICE	
OUT	R/W	Output Specification	OUTLINK	
PCOF	R/W	Proportional Gain	DOUBLE	
PERL	R/W	Periodic Limits	RECCHOICE	(0:"NO", 1:"YES")
POST	R/W	Post-move commands	STRING	
PP	R	Post process command	SHORT	
PREC	R/W	Display Precision	SHORT	
PREM	R/W	Pre-move commands	STRING	
RBV	R	User Readback Value	DOUBLE	
RCNT	R	Retry count	SHORT	
RDBD	R/W	Retry Deadband (EGU)	DOUBLE	
SPDB	R/W	Set Point Deadband (EGU)	DOUBLE	
RDBL	R	Readback Location	INLINK	
RDIF	R	Difference rval-rrbv	LONG	
REP	R	Raw Encoder Position	DOUBLE	
RHLS	R	Raw High Limit Switch	SHORT	
RINP	R/W	RMP Input Link	INLINK	
RLLS	R	Raw Low Limit Switch	SHORT	
RLNK	R	Readback OutLink	OUTLINK	
RLV	R/W*	Relative Value	DOUBLE	
RMOD	R/W	Retry Mode	RECCHOICE	(0:"Unity", 1:"Arthmetic", 2:"Geometric", 3:"In-Position")
RMP	R	Raw Motor Position	DOUBLE	
RRBV	R	Raw Readback Value	DOUBLE	
RRES	R/W	Readback Step Size (EGU)	DOUBLE	
RTRY	R/W	Max retry count	SHORT	
RVAL	R/W*	Raw Desired Value	DOUBLE	
RVEL	R	Raw Velocity	LONG	
S	R/W	Speed (RPS)	DOUBLE	
SBAK	R/W	BL Speed (RPS)	DOUBLE	
SBAS	R/W	Base Speed (RPS)	DOUBLE	
SET	R/W	Set/Use Switch	RECCHOICE	(0:"Use", 1:"Set")
SMAX	R/W	Max Velocity (RPS)	DOUBLE	
SPMG	R/W*	Stop/Pause/Move/Go	RECCHOICE	(0:"Stop", 1:"Pause", 2:"Move", 3:"Go")
SREV	R/W*	Steps per Revolution	LONG	
SSET	R/W	Set SET Mode	SHORT	
STOO	R/W	STOP OutLink	OUTLINK	
STOP	R/W*	Stop	SHORT	
STUP	R/W*	Status Update Request	RECCHOICE	OFF(0), ON(1), BUSY(2)
SUSE	R/W	Set USE Mode	SHORT	
SYNC	R/W*	Sync positions	RECCHOICE	(0:"No", 1:"Yes")
TDIR	R	Direction of Travel	SHORT	
TWF	R/W*	Tweak motor Forward	SHORT	
TWR	R/W*	Tweak motor Reverse	SHORT	
TWV	R/W*	Tweak Step Size (EGU)	DOUBLE	
UEIP	R/W*	Use Encoder If Present	RECCHOICE	(0:"No", 1:"Yes")
UREV	R/W*	EGU's per Revolution	DOUBLE	
URIP	R/W*	Use RDBL Link If Present	RECCHOICE	(0:"No", 1:"Yes")
VAL	R/W*	User Desired Value	DOUBLE	
VBAS	R/W	Base Velocity (EGU/s)	DOUBLE	
VELO	R/W	Velocity (EGU/s)	DOUBLE	
VERS	R	Code Version	DOUBLE	e.g., "1.95"
VMAX	R/W	Max Velocity (EGU/s)	DOUBLE	
VOF	R/W	Variable Offset	SHORT	
Note: In the Access column above:
R	Read only	
R/W	Read and write are allowed
R/W*	Read and write are allowed; write triggers record processing if the record's SCAN field is set to "Passive."
N	No access allowed
Note: In the Prompt column above:
EGU	Engineering Units
RPS	Revolutions Per Second
*/
#define INITDRIVER_DEF
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(epicsMotor, 1.0.0, chaos::driver::actuator::epicsMotor)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::epicsMotor, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

// register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(epicsMotor)
CLOSE_REGISTER_PLUGIN

#ifndef ACTAPP
#define ACTAPP LAPP_ << "[epicsMotor] "
#define ACTERR LERR_ << "[epicsMotor] "
#define ACTDBG LDBG_ << "[epicsMotor] "
#endif
namespace chaos {
namespace driver {
namespace actuator {
// default constructor definition
epicsMotor::epicsMotor():counter(0) {
}

// default descrutcor
epicsMotor::~epicsMotor() {
}

void epicsMotor::driverInit(const chaos::common::data::CDataWrapper& json)  {
  int ret = -1;

  ACTDBG << "Configuration:" << json.getJSONString();
  // std::vector<std::string> pvlist ={"Pva1:Image"};
  // std::vector<std::string> pvlist ={IMAGE_ARRAY,"cam1:DataType_RBV","cam1:ColorMode_RBV"};
  chaos::common::data::CDWUniquePtr newconf = json.clone();

  std::map<std::string, std::string> pvprprop = {
    {"ACCL", "ACCELERATION"}, //R/W	Seconds to Velocity	DOUBLE	acceleration time
    {"ADEL",""},//	R/W	Archive Deadband	DOUBLE	Deadband on RBV archive monitor
    {"ALST",""},//	R	Last Valve Archived	DOUBLE	Last RBV value to be archived
{"ATHM",""},//	R	At HOME	SHORT	uses the HOME switch
{"BACC",""},//	R/W	BL Seconds to Veloc.	DOUBLE	backlash acceleration time
{"BDST",""},//	R/W	BL Distance (EGU)	DOUBLE	backlash distance
{"BVEL",""},//	R/W	BL Velocity (EGU/s)	DOUBLE	backlash speed
{"CARD",""},//	R	Card Number	SHORT	EPICS card #
{"CBAK",""},//	None	Callback structure	NOACCESS	
{"CDIR",""},//	R	Raw commanded direction	SHORT	(1:"Pos", 0:"Neg")
{"CNEN",""},//	R/W	Enable control	RECCHOICE	(0:"Disable", 1:"Enable")
{"DCOF",""},//	R/W	Derivative Gain	DOUBLE	
{"DHLM",""},//	R/W*	Dial High Limit	DOUBLE	
{"DIFF",""},//	R	Difference dval-drbv	DOUBLE	
{"DINP",""},//	R/W	DMOV Input Link	INLINK	
{"DIR",""},//	R/W*	User Direction	RECCHOICE	(0:"Pos", 1:"Neg")
{"DLLM",""},//	R/W*	Dial Low Limit	DOUBLE	
{"DLY",""},//	R/W	Readback settle time (s)	DOUBLE	
{"DMOV",""},//	R	Done moving to value	SHORT	The "done" flag
{"DOL",""},//	R	Desired Output Loc	INLINK	only for closed-loop mode
{"DRBV",""},//	R	Dial Readback Value	DOUBLE	
{"DVAL",""},//	R/W*	Dial Desired Value	DOUBLE	
{"EGU",""},//	R/W	Engineering Units	STRING	
{"ERES",""},//	R/W*	Encoder Step Size (EGU)	DOUBLE	
{"FOF",""},//	R/W	Freeze Offset	SHORT	
{"FOFF",""},//	R/W	Offset-Freeze Switch	RECCHOICE	(0:"Variable", 1:"Frozen")
{"FRAC",""},//	R/W	Move Fraction	DOUBLE	
{"HHSV",""},//	R/W*	Hihi Severity	GBLCHOICE	
{"HIGH",""},//	R/W*	High Alarm Limit	DOUBLE	
{"HIHI",""},//	R/W*	Hihi Alarm Limit	DOUBLE	
{"HLM",""},//	R/W*	User High Limit	DOUBLE	
{"HLS",""},//	R	At High Limit Switch	SHORT	
{"HLSV",""},//	R/W*	HW Lim. Violation Svr	GBLCHOICE	
{"HOMF","HOME_FORWARD"},//	R/W*	Home Forward	SHORT	
{"HOMR","HOME_REVERSE"},//	R/W*	Home Reverse	SHORT	
{"HOPR",""},//	R/W	High Operating Range	DOUBLE	
{"HSV",""},//	R/W*	High Severity	GBLCHOICE	
{"HVEL","HOME_SPEED"},//	R/W*	Home Velocity	DOUBLE	
{"ICOF",""},//	R/W	Integral Gain	DOUBLE	
{"IGSET",""},//	R/W	Ignore SET Field	SHORT	(0:Normal operation, 1: SET is ignored)
{"INIT",""},//	R/W	Startup commands	STRING	
{"JAR",""},//	R/W	Jog Acceleration (EGU/s^2)	DOUBLE	
{"JOGF",""},//	R/W*	Jog motor Forward	SHORT	careful!
{"JOGR",""},//	R/W*	Jog motor Reverse	SHORT	careful!
{"JVEL",""},//	R/W	Jog Velocity	DOUBLE	
{"LDVL",""},//	R	Last Dial Des Val	DOUBLE	
{"LLM",""},//	R/W*	User Low Limit	DOUBLE	
{"LLS",""},//	R	At Low Limit Switch	SHORT	
{"LLSV",""},//	R/W*	Lolo Severity	GBLCHOICE	
{"LOCK",""},//	R/W*	Soft Channel Position Lock	RECCHOICE	(0:"NO", 1:"YES")
{"LOLO",""},//	R/W*	Lolo Alarm Limit	DOUBLE	
{"LOPR",""},//	R/W	Low Operating Range	DOUBLE	
{"LOW",""},//	R/W*	Low Alarm Limit	DOUBLE	
{"LRLV",""},//	R	Last Rel Value	DOUBLE	
{"LRVL",""},//	R	Last Raw Des Val	DOUBLE	
{"LSPG",""},//	R	Last SPMG	RECCHOICE	(See SPMG)
{"LSV",""},//	R/W*	Low Severity	GBLCHOICE	
{"LVAL",""},//	R	Last User Des Val	DOUBLE	
{"LVIO",""},//	R	Limit violation	SHORT	
{"MDEL",""},//	R/W	Monitor Deadband	DOUBLE	Deadband on RBV valve monitor
{"MLST",""},//	R	Last Valve Monitored	DOUBLE	Last RBV value to be value monitored
{"MIP",""},//	R	Motion In Progress	SHORT	
{"MISS",""},//	R	Ran out of retries	SHORT	
{"MMAP",""},//	R	Monitor Mask	ULONG	
{"MOVN",""},//	R	Motor is moving	SHORT	Don't confuse with DMOV
{"MRES",""},//	R/W*	Motor Step Size (EGU)	DOUBLE	
{"MSTA",""},//	R	Motor Status	ULONG	
{"NMAP",""},//	R	Monitor Mask	ULONG	
{"NTM",""},//	R/W*	New Target Monitor	RECCHOICE	(0:"NO", 1:"YES")
{"NTMF",""},//	R/W*	New Target Monitor Deadband Factor	SHORT	Determines NTM deadband; NTMF >= 2
{"OFF",""},//	R/W	User Offset (EGU)	DOUBLE	
{"OMSL",""},//	R/W	Output Mode Select	GBLCHOICE	
{"OUT",""},//	R/W	Output Specification	OUTLINK	
{"PCOF",""},//	R/W	Proportional Gain	DOUBLE	
{"PERL",""},//	R/W	Periodic Limits	RECCHOICE	(0:"NO", 1:"YES")
{"POST",""},//	R/W	Post-move commands	STRING	
{"PP",""},//	R	Post process command	SHORT	
{"PREC",""},//	R/W	Display Precision	SHORT	
{"PREM",""},//	R/W	Pre-move commands	STRING	
{"RBV",""},//	R	User Readback Value	DOUBLE	
{"RCNT",""},//	R	Retry count	SHORT	
{"RDBD",""},//	R/W	Retry Deadband (EGU)	DOUBLE	
{"SPDB",""},//	R/W	Set Point Deadband (EGU)	DOUBLE	
{"RDBL",""},//	R	Readback Location	INLINK	
{"RDIF",""},//	R	Difference rval-rrbv	LONG	
{"REP",""},//	R	Raw Encoder Position	DOUBLE	
{"RHLS",""},//	R	Raw High Limit Switch	SHORT	
{"RINP",""},//	R/W	RMP Input Link	INLINK	
{"RLLS",""},//	R	Raw Low Limit Switch	SHORT	
{"RLNK",""},//	R	Readback OutLink	OUTLINK	
{"RLV",""},//	R/W*	Relative Value	DOUBLE	
{"RMOD",""},//	R/W	Retry Mode	RECCHOICE	(0:"Unity", 1:"Arthmetic", 2:"Geometric", 3:"In-Position")
{"RMP",""},//	R	Raw Motor Position	DOUBLE	
{"RRBV",""},//	R	Raw Readback Value	DOUBLE	
{"RRES",""},//	R/W	Readback Step Size (EGU)	DOUBLE	
{"RTRY",""},//	R/W	Max retry count	SHORT	
{"RVAL",""},//	R/W*	Raw Desired Value	DOUBLE	
{"RVEL",""},//	R	Raw Velocity	LONG	
{"S","SPEED"},//	R/W	Speed (RPS)	DOUBLE	
{"SBAK",""},//	R/W	BL Speed (RPS)	DOUBLE	
{"SBAS","VELOCITY_BASE_RPS"},//	R/W	Base Speed (RPS)	DOUBLE	
{"SET",""},//	R/W	Set/Use Switch	RECCHOICE	(0:"Use", 1:"Set")
{"SMAX","MAX_VELOCITY_RPS"},//	R/W	Max Velocity (RPS)	DOUBLE	
{"SPMG",""},//	R/W*	Stop/Pause/Move/Go	RECCHOICE	(0:"Stop", 1:"Pause", 2:"Move", 3:"Go")
{"SREV",""},//	R/W*	Steps per Revolution	LONG	
{"SSET",""},//	R/W	Set SET Mode	SHORT	
{"STOO",""},//	R/W	STOP OutLink	OUTLINK	
{"STOP",""},//	R/W*	Stop	SHORT	
{"STUP",""},//	R/W*	Status Update Request	RECCHOICE	OFF(0), ON(1), BUSY(2)
{"SUSE",""},//	R/W	Set USE Mode	SHORT	
{"SYNC",""},//	R/W*	Sync positions	RECCHOICE	(0:"No", 1:"Yes")
{"TDIR",""},//	R	Direction of Travel	SHORT	
{"TWF",""},//	R/W*	Tweak motor Forward	SHORT	
{"TWR",""},//	R/W*	Tweak motor Reverse	SHORT	
{"TWV",""},//	R/W*	Tweak Step Size (EGU)	DOUBLE	
{"UEIP","USE_ENCODER"},//	R/W*	Use Encoder If Present	RECCHOICE	(0:"No", 1:"Yes")
{"UREV",""},//	R/W*	EGU's per Revolution	DOUBLE	
{"URIP",""},//	R/W*	Use RDBL Link If Present	RECCHOICE	(0:"No", 1:"Yes")
{"VAL",""},//	R/W*	User Desired Value	DOUBLE	
{"VBAS","VELOCITY_BASE_EGUS"},//	R/W	Base Velocity (EGU/s)	DOUBLE	
{"VELO","VELOCITY_EGUS"},//	R/W	Velocity (EGU/s)	DOUBLE	
{"VERS",""},//	R	Code Version	DOUBLE	e.g., "1.95"
{"VMAX","MAX_VELOCITY_EGU"},//	R/W	Max Velocity (EGU/s)	DOUBLE	
{"VOF",""}};//	R/W	Variable Offset	SHORT	


  // R/W* Read and write are allowed; write triggers record processing if the record's SCAN field is set to "Passive."
  ::driver::epics::common::EpicsGenericDriver::addPVListConfig(*(newconf.get()), pvprprop);

    devicedriver = new ::driver::epics::common::EpicsPVAccessDriver(*(newconf.get()));
    if(devicedriver==NULL){
          throw chaos::CException(-1, "Cannot initialize Driver with params:"+ json.getJSONString(), __PRETTY_FUNCTION__);

    }
    ACTDBG << "Init driver initialization with json " << json.getJSONString().c_str();
 
 //   stopMotion(-1); // abort all movements (if any)
  
  

}

void epicsMotor::driverInit(const char* initParameter)  {
  chaos::common::data::CDataWrapper cd;
  cd.setSerializedJsonData(initParameter);
  driverInit(cd);
}




int epicsMotor::setParameter(int axisID, std::string parName, std::string value) {
    int ret;
    std::string rets = "";


    if ((parName == "SPEED") || (parName == "HVEL")|| (parName =="ACCL"))
    {
        devicedriver->write(parName,atof(value.c_str()));

    }
    else if ((parName == "HOMR") || (parName == "HOMF")){
      // home reverse, home forward
      devicedriver->write(parName,atoi(value.c_str()));

    }
    


  return 0;
}
int epicsMotor::moveRelative(int axisID, double mm) {
    int ret;
   
   if(mm>0){
    devicedriver->write("TWV",mm);
    devicedriver->write("TWF",1);
   } else {
     devicedriver->write("TWV",-mm);
      devicedriver->write("TWR",1);
   }
  return 0;
}
int epicsMotor::getParameter(int axisID, std::string parName, std::string& resultString) {
    int ret;
   
    
  return 0;
}
int epicsMotor::getPosition(int axisID, readingTypes mode, double* deltaPosition_mm) {
  double val;
  int ret=devicedriver->read("VAL",val);
  if(deltaPosition_mm){
    *deltaPosition_mm=val;
  } else {
    ACTERR<<" no valid pointer";
    return -1;
  }

  return (ret>0)?0:ret;
}
int epicsMotor::initACT(int axisID, void* par) {
  std::string version;
  int ret;
  if((ret=getHWVersion(axisID,version))<=0){
    ACTERR<<" cannot retrieve version ret:"<<ret;
    return ret;
  }
  ACTDBG << "Init Done, version:" << version<<std::endl;
  return 0;
}
int epicsMotor::configAxis(int axisID, void* initialization_string) {
    auto search = statusMap.find(axisID);
    if (search != statusMap.end()) {
        ACTERR << "PROBLEM. Already configured axis " << axisID << '\n';
        //<< search->first << " " << search->second << '\n';
    }
    else {
        //statusMap.insert(axisID);
        statusMap[axisID] = (uint64_t) ::common::actuators::actuatorStatus::ACTUATOR_READY;

        /*for (const auto& [key, value] : statusMap) {
            ACTAPP << "map now:" << '[' << key << "] = " << value << "; ";*/
    }
          
  return 0;
}
int epicsMotor::deinitACT(int axisID) {
  stopMotion(axisID);
    statusMap.erase(axisID);
  return 0;
}
int epicsMotor::hardreset(int axisID, bool mode) {
  return 0;
}

int epicsMotor::getSWVersion(int axisID, std::string& version) {

  return  getHWVersion(axisID,version);

}
int epicsMotor::getHWVersion(int axisID, std::string& version) {
  double ver=-1;
    int ret=devicedriver->read("VERS",ver);
  std::stringstream ss;
  ss<<ver;
  version=ss.str();
  return (ret>0)?0:ret;
}
int epicsMotor::listParameters(std::string& dataset) {
    
  return 0;
}
int epicsMotor::stopMotion(int axisID) {

  int ret;
  ret=devicedriver->write("STOP",1);
  return (ret>0)?0:ret;;
  
}
int epicsMotor::homing(int axisID, homingType mode) {
   int ret=devicedriver->write("HOMF",1);
   if(ret>0){
    ret=devicedriver->write("HOMR",1);
   }
  return (ret>0)?0:ret;
}
int epicsMotor::soft_homing(int axisID, double positionToSet) {
    

  return 0;
}
int epicsMotor::getState(int axisID, int* _state, std::string& desc) {
    int32_t state;
    int ret;
    uint64_t lastVal= statusMap[axisID];
    int stat;
    ret=devicedriver->read("MSTA",state);

    
    /*
  1  DIRECTION: last raw direction; (0:Negative, 1:Positive)
  2 DONE: motion is complete.
  3 PLUS_LS: plus limit switch has been hit.
  4 HOMELS: state of the home limit switch.
  5 Unused
  6 POSITION: closed-loop position control is enabled.
  7 SLIP_STALL: Slip/Stall detected (eg. fatal following error)
  8 HOME: if at home position.
  9 PRESENT: encoder is present.
 10 PROBLEM: driver stopped polling, or hardware problem
 11 MOVING: non-zero velocity present.
 12 GAIN_SUPPORT: motor supports closed-loop position control.
 13 COMM_ERR: Controller communication error.
 14 MINUS_LS: minus limit switch has been hit.
15 HOMED: the motor has been homed.
    */
    if(devicedriver->read("SPMG",stat)){
      if(stat==3){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED;
      } else {
        lastVal &= ::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED;

      }
    }

    if(state&(1<<2)){

        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_MOTION_COMPLETE;
    } else {
        lastVal &= ~(::common::actuators::actuatorStatus::ACTUATOR_MOTION_COMPLETE);
    }
    if(state&(1<<3)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_LSP_LIMIT_ACTIVE;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_LSP_LIMIT_ACTIVE);  
    }

     if(state&(1<<4)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_HOME_LIMIT;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_HOME_LIMIT);  
    }
     if(state&(1<<6)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_POSITION_CTRL;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_POSITION_CTRL);  
    }
     if(state&(1<<7)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_STALL_ERROR;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_STALL_ERROR);  
    }
    if(state&(1<<8)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_AT_HOME;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_AT_HOME);  
    }
    if(state&(1<<9)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_ENCODER;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_ENCODER);  
    }
     if(state&(1<<10)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_FAULT;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_FAULT);  
    }
    if(state&(1<<11)){

        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_INMOTION;
    } else {
        lastVal &= ~(::common::actuators::actuatorStatus::ACTUATOR_INMOTION);

    }
    if(state&(1<<12)){

        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_GAIN_CTRL;
    } else {
        lastVal &= ~(::common::actuators::actuatorStatus::ACTUATOR_GAIN_CTRL);

    }
    if(state&(1<<13)){

        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_COMM_ERROR;
    } else {
        lastVal &= ~(::common::actuators::actuatorStatus::ACTUATOR_COMM_ERROR);

    }
    if(state&(1<<14)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_LSN_LIMIT_ACTIVE;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_LSN_LIMIT_ACTIVE);  
    }
    if(state&(1<<15)){
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_HOMED;
    } else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_HOMED);  
    }
      
    statusMap[axisID] = lastVal;
    *_state = lastVal;
    desc = AbstractActuator::StatusDecoding(statusMap[axisID]);

  return 0;
}
int epicsMotor::getAlarms(int axisID, uint64_t* alrm, std::string& desc) {
    *alrm = 0;
    desc = "No Alarm";

    if(statusMap[axisID]&::common::actuators::actuatorStatus::ACTUATOR_FAULT){
      *alrm = ::common::actuators::actuatorStatus::ACTUATOR_FAULT;
      desc="Fault|";
    }
     if(statusMap[axisID]&::common::actuators::actuatorStatus::ACTUATOR_COMM_ERROR){
      *alrm |= ::common::actuators::actuatorStatus::ACTUATOR_COMM_ERROR;
      desc+="Communication Error|";

    }
    if(statusMap[axisID]&::common::actuators::actuatorStatus::ACTUATOR_STALL_ERROR){
      *alrm |= ::common::actuators::actuatorStatus::ACTUATOR_STALL_ERROR;
      desc+="Stall Error";

    }
  return 0;
}
int epicsMotor::resetAlarms(int axisID, uint64_t alrm) {
  return 0;
}
int epicsMotor::poweron(int axisID, int on) {
    uint64_t lastVal = statusMap[axisID];
    int ret;
    if (on == 1){
      ret=devicedriver->write("SPMG",3);
      lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED;
    }else{
      ret=devicedriver->write("SPMG",0);

      lastVal &= (~(::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED));
    }
    
    statusMap[axisID] = lastVal;
  return  (ret>0)?0:ret;
}
uint64_t epicsMotor::getFeatures() {
  return 0;
}
int epicsMotor::moveAbsolute(int axisID, double mm) {
  int ret=devicedriver->write("VAL",mm);
  

  return (ret>0)?0:ret;
}
}  // namespace actuator
}  // namespace driver
}  // namespace chaos