#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#ifndef SIMULATOR
#include "serial_data.h"

extern volatile serialData_t serialData;
extern volatile bool data_ready;
extern volatile bool data_reading;
#endif


Model::Model() : modelListener(0)
{

}

void Model::tick()
{
#ifndef SIMULATOR
if(data_ready) {
	data_reading = true;
	modelListener->process_uart(serialData);
	data_reading = false;
}
#endif
}
