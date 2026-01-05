/* serial_data.h
 *
 *  Created on: Dec 27, 2025
 *      Author: jainp
 */

#ifndef INC_SERIAL_DATA_H_
#define INC_SERIAL_DATA_H_
#include <stdbool.h>
typedef struct  {
 float batt_volt;
 float batt_current;
 float soc;
 float soh;
 float cell_volt_max;
 float cell_volt_min;
 float cell_temp_max;
 float cell_temp_min;
 float mos_temp;
 float ambient_temp;
 float pack_capacity;
 float remaining_capacity;
}serialData_t;

#endif /* INC_SERIAL_DATA_H_ */