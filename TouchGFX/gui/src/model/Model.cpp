#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "serial_data.h"

extern volatile serialData_t serialData;
extern volatile bool data_ready;
extern volatile bool data_reading;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	if(data_ready) {
		data_reading = true;
		modelListener->process_uart(serialData);
		data_reading = false;
	}

}
