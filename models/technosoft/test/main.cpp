


#include <chaos/common/chaos_constants.h>
#include <chaos/cu_toolkit/ChaosCUToolkit.h>
#include <chaos/common/exception/CException.h>

/*** CU Types ****/

#include<driver/actuator/models/technosoft/TechnosoftMotor.h>
#include<driver/actuator/core/SCActuatorControlUnit.h>



int main(int argc,char**argv){
	try{
		chaos::cu::ChaosCUToolkit::getInstance()->init(argc, argv);
		
		REGISTER_CU(::driver::actuator::SCActuatorControlUnit);
		REGISTER_DRIVER(chaos::driver::actuator,TechnosoftMotor);


		chaos::cu::ChaosCUToolkit::getInstance()->start();
	} catch (chaos::CException& e) {
		std::cerr<<"Exception:"<<std::endl;
		std::cerr<< "domain	:"<<e.errorDomain << std::endl;
		std::cerr<< "cause	:"<<e.errorMessage << std::endl;return -1;
	} catch (program_options::error &e){
		std::cerr << "Unable to parse command line: " << e.what() << std::endl;return -2;
	} catch (...){
		std::cerr << "unexpected exception caught.. " << std::endl;return -3;
	}
        return 0;
}

