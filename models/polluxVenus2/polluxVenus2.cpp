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
#include "polluxVenus2.h"
#include <chaos/cu_toolkit/driver_manager/driver/AbstractDriverPlugin.h>
#include <common/actuators/core/AbstractActuator.h>

#define INITDRIVER_DEF
// GET_PLUGIN_CLASS_DEFINITION
// we need only to define the driver because we don't are makeing a plugin

OPEN_CU_DRIVER_PLUGIN_CLASS_DEFINITION(polluxVenus2, 1.0.0, chaos::driver::actuator::polluxVenus2)
REGISTER_CU_DRIVER_PLUGIN_CLASS_INIT_ATTRIBUTE(chaos::driver::actuator::polluxVenus2, http_address / dnsname
                                               : port)
CLOSE_CU_DRIVER_PLUGIN_CLASS_DEFINITION

// register the two plugin
OPEN_REGISTER_PLUGIN
REGISTER_PLUGIN(polluxVenus2)
CLOSE_REGISTER_PLUGIN

#ifndef ACTAPP
#define ACTAPP LAPP_ << "[polluxVenus2] "
#define ACTERR LERR_ << "[polluxVenus2] "
#define ACTDBG LDBG_ << "[polluxVenus2] "
#endif
namespace chaos {
namespace driver {
namespace actuator {
// default constructor definition
polluxVenus2::polluxVenus2():counter(0) {
}

// default descrutcor
polluxVenus2::~polluxVenus2() {
}
std::string polluxVenus2::getAnswer(int timeo_ms){
  
  //int ret=serial->readLine(ss,POLLUX_TERMINATOR,timeo_ms);
  char buff[256];
  int ret = serial->read(buff,256,timeo_ms);
  if(ret<0){
    ACTERR<<"Timeout answer";
  }
  std::string ss(buff);
  return ss;
}
int polluxVenus2::abortAll(){
  char ctrlc=2;
  ACTDBG << "Abort ALL";

  return serial->write((void*)&ctrlc,1);
}

int polluxVenus2::sendCommand(const std::string param, int axis, const std::string& cmd) {
    std::stringstream ss;
    ss << param << " " << axis << " " << cmd;
    ACTDBG << (++counter) << "] Sending \"" << ss.str() << "\"";
    ss << POLLUX_TERMINATOR;
    int ret = serial->write((void*)ss.str().c_str(), ss.str().size());
    return ret;
}

int polluxVenus2::sendCommand(int axis,const std::string&cmd){
    std::stringstream ss;
    ss<<axis<<" "<<cmd;
    ACTDBG << (++counter)<<"] Sending \"" << ss.str()<<"\"";

    ss<<POLLUX_TERMINATOR;
   int ret=serial->write((void*)ss.str().c_str(),ss.str().size());
   return ret;
}

int polluxVenus2::sendCommand(int axis,const std::string&cmd,std::string&reply){
  ChaosLockGuard a(io_mux);
  reply="";
  int ret=sendCommand(axis,cmd);
  if(ret<0){
    getAnswer();
    return ret;
  }
  reply=getAnswer();
  ACTDBG << counter<<"] Reply \"" << reply<<"\"";

  return reply.size();
}

void polluxVenus2::driverInit(const chaos::common::data::CDataWrapper& json) throw(chaos::CException) {
  int ret = -1;
  ACTDBG << "Init driver initialization with json " << json.getJSONString().c_str();
  if(!json.hasKey("driver_param")){
    throw chaos::CException(-1, "Missing driver_param key", "polluxVenus2::driverInit");

  }
  chaos::common::data::CDWUniquePtr channel=json.getCSDataValue("driver_param");
  serial = ::common::serial::SerialChannelFactory::getChannel(*channel.get());

  if (serial.get() == NULL) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(1, "Bad parameters for polluxVenus2:"+channel->getJSONString(), "polluxVenus2::driverInit");
  } else {
  }

  if ((ret = serial->init()) != 0) {
    ACTERR << "Init Failed! ret:" << ret << std::endl;
    throw chaos::CException(-2, "Cannot connect", "polluxVenus2::driverInit");
  }
  stopMotion(-1); // abort all movements (if any)
  
  

}

void polluxVenus2::driverInit(const char* initParameter) throw(chaos::CException) {
  // rett= motor->getPosition((::common::actuators::AbstractActuator::readingTypes)1,&mmpos);
}

std::string formatDouble(double val)
{
    char buf[32];
    sprintf(buf, "%.6f", val);
    return std::string(buf);
}

std::string formatDouble(std::string sval)
{
    double val = atof(sval.c_str());
    return formatDouble(val);

}



int polluxVenus2::setParameter(int axisID, std::string parName, std::string value) {
    int ret;
    std::string rets = "";


    if (parName == "speed")
    {
        std::string vv = formatDouble(value);
        if ((ret = sendCommand(vv, axisID, "snv")) <= 0) {
            return -1;
        }
    }
    else if ((parName == "hwminpos") || (parName == "hwmaxpos"))
    {
    }
    else if (parName == "highspeedhoming")
    {
        std::string vv = formatDouble(value);
        if ((ret = sendCommand(vv+ " 1 ", axisID, "sncalvel")) <= 0) {
            return -1;
        }
    }
    else if (parName == "acceleration")
    {
        std::string vv = formatDouble(value);
        if ((ret = sendCommand(vv, axisID, "sna")) <= 0) {
            return -1;
        }
    }
    else if (parName == "pitch") {
        std::string vv = formatDouble(value);
        if ((ret = sendCommand(vv, axisID, "setpitch")) <= 0) {
            return -1;
        }
    }


  return 0;
}
int polluxVenus2::moveRelative(int axisID, double mm) {
    int ret;
   
    std::string A = formatDouble(mm);
    if ((ret = sendCommand(A, axisID, "nr")) <= 0) {
        return -1;
    }
  return 0;
}
int polluxVenus2::getParameter(int axisID, std::string parName, std::string& resultString) {
    int ret;
    std::string rets="";
    
   
    if (parName == "speed")
    {
        if ((ret = sendCommand(axisID, "gnv", rets)) <= 0) {
            return -1;
        }
        resultString= rets;
    }
    else if ((parName == "hwminpos") || (parName == "hwmaxpos"))
    {
        if ((ret = sendCommand(axisID, "getnlimit", rets)) <= 0) {
            return -1;
        }
        float ll, hl;
        if (sscanf(rets.c_str(), "%f%f", &ll, &hl) != 2) {
            return -1;
        }
        ACTDBG << "Axis " << axisID << " " << parName<< " " << rets;
        resultString = (parName == "hwminpos") ? std::to_string(ll) : std::to_string(hl);
    }
    else if (parName == "highspeedhoming") {
        
        if ((ret = sendCommand(axisID, "getncalvel", rets)) <= 0) {
            return -1;
        }
        float ll, hl;
        if (sscanf(rets.c_str(), "%f%f", &ll, &hl) != 2) {
            return -1;
        }
        resultString = std::to_string(ll);
    }
    else if (parName == "acceleration")
    {
        if ((ret = sendCommand(axisID, "gna", rets)) <= 0) {
            return -1;
        }
        ACTDBG << "Axis " << axisID << " acceleration:" << rets;
        resultString = rets;
    }
    else if (parName == "pitch") {
        
        if ((ret = sendCommand(axisID, "getpitch", rets)) <= 0) {
            return -1;
        }
        ACTDBG << "Axis " << axisID << " pitch:" << rets;
        resultString = rets;
    }
        ;

    
  return 0;
}
int polluxVenus2::getPosition(int axisID, readingTypes mode, double* deltaPosition_mm) {
  std::string rets;
  int ret;
  if((ret=sendCommand(axisID,"npos",rets))<=0){
    return -1;
  } 
  *deltaPosition_mm=atof(rets.c_str());
  //ACTDBG << "Axis " << axisID<<" position:"<<*deltaPosition_mm;

  return 0;
}
int polluxVenus2::initACT(int axisID, void* par) {
  sendCommand(axisID,"nclear"); // clear fifo parameter stack
  std::string version;
  int ret;
  if((ret=getHWVersion(axisID,version))<=0){
    ACTERR<<" cannot retrieve version ret:"<<ret;
    return ret;
  }
  ACTDBG << "Init Done, version:" << version<<std::endl;
  return 0;
}
int polluxVenus2::configAxis(int axisID, void* initialization_string) {
    auto search = this->statusMap.find(axisID);
    if (search != this->statusMap.end()) {
        ACTERR << "PROBLEM. Already configured axis " << axisID << '\n';
        //<< search->first << " " << search->second << '\n';
    }
    else {
        //this->statusMap.insert(axisID);
        this->statusMap[axisID] = (uint64_t) ::common::actuators::actuatorStatus::ACTUATOR_READY;

        /*for (const auto& [key, value] : this->statusMap) {
            ACTAPP << "map now:" << '[' << key << "] = " << value << "; ";*/
    }
          
  return 0;
}
int polluxVenus2::deinitACT(int axisID) {
    this->statusMap.erase(axisID);
  return 0;
}
int polluxVenus2::hardreset(int axisID, bool mode) {
  return 0;
}

int polluxVenus2::getSWVersion(int axisID, std::string& version) {
  return  sendCommand(axisID,"nversion",version);

}
int polluxVenus2::getHWVersion(int axisID, std::string& version) {
  return sendCommand(axisID,"nidentify",version);

}
int polluxVenus2::listParameters(std::string& dataset) {
    dataset.clear();
    dataset = "{\"attributes\":[";
    dataset += "{\"name\":\"speed\",\"description\":\"Max speed of trapezoidal "
        "profile\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += ",";
    dataset += "{\"name\":\"acceleration\",\"description\":\"Acceleration of "
        "trapezoidal "
        "profile\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += ",";
    dataset += "{\"name\":\"pitch\",\"description\":\"The distance / revolution ratio (mm)\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += ",";
    dataset += "{\"name\":\"highspeedhoming\",\"description\":\"Max speed used for homing procedure\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += ",";
    dataset += "{\"name\":\"hwminpos\",\"description\":\"The minimum allowed hw position\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += ",";
    dataset += "{\"name\":\"hwmaxpos\",\"description\":\"The maximum allowed hw position\",\"datatype\":\"double\",\"direction\":\"Input\" }";
    dataset += "]}";
  return 0;
}
int polluxVenus2::stopMotion(int axisID) {

  if(axisID==-1){
    ACTDBG << "ABORT ALL AXIS";

    return abortAll();
  } 
  int ret;
  if ((ret = sendCommand(axisID, "nabort")) <= 0) {
      return -1;
  }
  uint64_t state = this->statusMap[axisID];
  //end homing
  state &= (~::common::actuators::actuatorStatus::HOMING_IN_PROGRESS);
  this->statusMap[axisID] = state;
  return 0;
  
}
int polluxVenus2::homing(int axisID, homingType mode) {
    uint64_t state = this->statusMap[axisID];
    bool homing_in_progress = (state & ::common::actuators::actuatorStatus::HOMING_IN_PROGRESS) != 0;
    bool calibrationSwitchPressed;
    std::string command;
    int ret;
    std::string rets;
    //getswst
    if ((ret = sendCommand(axisID, "getswst", rets)) <= 0) {
        return -1;
    }

    int nl = 0, pl = 0;
    if (sscanf(rets.c_str(), "%d%d", &nl, &pl) != 2)
    {
        ACTERR << "BAD ANSWER IN HOMING READING SWITCHES: "<< rets;
        return -1;
    }
    int calSwitch = (mode >= 0) ? nl : pl;
    command = (mode >= 0) ? "ncal" : "nrm";
    calibrationSwitchPressed = (calSwitch == 1);

    if (!homing_in_progress)
    {
        int ret;
        if ((ret = sendCommand(axisID, command.c_str())) <= 0) {
            return -1;
        }
        state |= ::common::actuators::actuatorStatus::HOMING_IN_PROGRESS;
        this->statusMap[axisID] = state;
        return 0;
    }
    else
    {
        if (calibrationSwitchPressed)
        {
            //end homing
            state &= (~::common::actuators::actuatorStatus::HOMING_IN_PROGRESS);
            this->statusMap[axisID] = state;
            return 0;
        }
        else
            return 1;
    }

  return 0;
}
int polluxVenus2::soft_homing(int axisID, double positionToSet) {
    int ret;
    std::string sval = formatDouble(-positionToSet);
    if ((ret = sendCommand(sval, axisID, "setnpos")) <= 0) {
        return -1;
    }

  return 0;
}
int polluxVenus2::getState(int axisID, int* state, std::string& desc) {
    
    uint64_t lastVal= this->statusMap[axisID];
    std::string rets;
    int ret;
    if ((ret = sendCommand(axisID, "nst", rets)) <= 0) {
        return -1;
    }
    if ((atoi(rets.c_str()) & 1) != 0) {
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_INMOTION;
    }
    
    else {
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_INMOTION);
    }
    //getswst
    if ((ret = sendCommand(axisID, "getswst", rets)) <= 0) {
        return -1;
    }
    
    int nl=0, pl=0;
    if (sscanf(rets.c_str(), "%d%d", &nl, &pl) != 2)
    {
        ACTERR << "BAD ANSWER IN GET_STATE while asking for limit switches status";
        return -1;
    }
    if (nl != 0)
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_LSN_LIMIT_ACTIVE;
    else
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_LSN_LIMIT_ACTIVE);
    
    if (pl != 0)
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_LSP_LIMIT_ACTIVE;
    else
        lastVal &= (~::common::actuators::actuatorStatus::ACTUATOR_LSN_LIMIT_ACTIVE);
    
    
    this->statusMap[axisID] = lastVal;
    *state = lastVal;
    desc = AbstractActuator::StatusDecoding(this->statusMap[axisID]);

  return 0;
}
int polluxVenus2::getAlarms(int axisID, uint64_t* alrm, std::string& desc) {
    *alrm = 0;
    desc = "No Alarm";
  return 0;
}
int polluxVenus2::resetAlarms(int axisID, uint64_t alrm) {
  return 0;
}
int polluxVenus2::poweron(int axisID, int on) {
    uint64_t lastVal = this->statusMap[axisID];

    if (on == 1)
        lastVal |= ::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED;
    else
        lastVal &= (~(::common::actuators::actuatorStatus::ACTUATOR_POWER_SUPPLIED));
    this->statusMap[axisID] = lastVal;
  return 0;
}
uint64_t polluxVenus2::getFeatures() {
  return 0;
}
int polluxVenus2::moveAbsolute(int axisID, double mm) {
    int ret;
    std::string A = formatDouble(mm);
    if ((ret = sendCommand(A, axisID, "nm")) <= 0) {
        return -1;
    }

  return 0;
}
}  // namespace actuator
}  // namespace driver
}  // namespace chaos