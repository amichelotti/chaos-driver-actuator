/*
 *	SCActuatorControlUnit.h
 *	!CHOAS
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
#ifndef __Actuator__SCActuatorControlUnit__
#define __Actuator__SCActuatorControlUnit__

#include <chaos/cu_toolkit/control_manager/SCAbstractControlUnit.h>
#include <driver/actuator/core/ChaosActuatorInterface.h>

using namespace chaos;
namespace driver {
namespace actuator {

class SCActuatorControlUnit : public chaos::cu::control_manager::SCAbstractControlUnit {
	PUBLISHABLE_CONTROL_UNIT_INTERFACE(SCActuatorControlUnit)

					// init parameter
					  std::string device_hw;

	chaos::driver::actuator::ChaosActuatorInterface *actuator_drv;

	bool waitOnCommandID(uint64_t command_id);
protected:
	/*
			 Define the Control Unit Dataset and Actions
	 */
	int  decodeType(const std::string& str_type, DataType::DataType& attribute_type) ;
	void unitDefineActionAndDataset()throw(chaos::CException);

	void unitDefineCustomAttribute();

	/*(Optional)
			 Initialize the Control Unit and all driver, with received param from MetadataServer
	 */
	void unitInit() throw(chaos::CException);
	/*(Optional)
			 Execute the work, this is called with a determinated delay, it must be as fast as possible
	 */
	void unitStart() throw(chaos::CException);
	/*(Optional)
			 The Control Unit will be stopped
	 */
	void unitStop() throw(chaos::CException);
	/*(Optional)
			 The Control Unit will be deinitialized and disposed
	 */
	void unitDeinit() throw(chaos::CException);

	//!restore method for actuators
	bool unitRestoreToSnapshot(chaos::cu::control_manager::AbstractSharedDomainCache * const snapshot_cache) throw(CException);

	//-----------utility methdo for the restore operation---------
	bool setPosition(double pos,bool sync = true);
	bool setPowerOn(int32_t value,bool sync = true);

	//-----------utility methdo for the restore operation---------
	bool moveAt(const std::string &name,double value,uint32_t size);
	bool setPower(const std::string &name,bool value,uint32_t size);

	const uint32_t *axID;


public:
	/*
			 Construct a new CU with an identifier
	 */
	SCActuatorControlUnit(const std::string& _control_unit_id,
			const std::string& _control_unit_param,
			const ControlUnitDriverList& _control_unit_drivers);

	/*
			 Base destructor
	 */
	~SCActuatorControlUnit();
	/*handlers*/
	//bool setSpeed(const std::string &name ,double value,size_t size);
	//bool setAcceleration(const std::string &name ,double value,size_t size);
	//bool setMovement(const std::string &name ,int32_t value,size_t size);
};
}
}

#endif /* defined(__Actuator__SCActuatorControlUnit__) */
