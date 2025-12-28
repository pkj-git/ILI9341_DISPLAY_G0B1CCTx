#include <gui/containers/CustomContainer1.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#ifndef SIMULATOR
#include "serial_data.h"
extern serialData_t data_screen1_copy;
#endif
#include <gui/common/SprintFloat.hpp>
CustomContainer1::CustomContainer1()
{

}

void CustomContainer1::initialize()
{
    CustomContainer1Base::initialize();
}
static constexpr uint8_t front_page_text_size{9U};
static constexpr TEXTS front_page_text_indx[front_page_text_size] = {
	    T_BATT_VOLT,
	    T_BATT_CURRENT,
	    T_SOC,
	    T_CELL_MAX_VOLT,
	    T_CELL_MIN_VOLT,
	    T_CELL_MAX_TEMP,
	    T_CELL_MIN_TEMP,
	    T_PACK_CAPACITY,
	    T_REMAINING_CAPACITY,
};
static constexpr uint8_t cell_voltage_page_text_size{16U};
static constexpr TEXTS cell_voltage_page_text_indx[cell_voltage_page_text_size] = {
	    T_CELL_VOLT_1,
	    T_CELL_VOLT_2,
	    T_CELL_VOLT_3,
	    T_CELL_VOLT_4,
	    T_CELL_VOLT_5,
	    T_CELL_VOLT_6,
	    T_CELL_VOLT_7,
	    T_CELL_VOLT_8,
	    T_CELL_VOLT_9,
	    T_CELL_VOLT_10,
	    T_CELL_VOLT_11,
	    T_CELL_VOLT_12,
	    T_CELL_VOLT_13,
	    T_CELL_VOLT_14,
	    T_CELL_VOLT_15,
	    T_CELL_VOLT_16
};

static constexpr uint8_t temperature_page_text_size{6U};
static constexpr TEXTS temperature_page_text_indx[cell_voltage_page_text_size] = {
	    T_TEMP_1,
	    T_TEMP_2,
	    T_TEMP_3,
	    T_TEMP_4,
	    T_MOS_TEMP,
	    T_AMB_TEMP,
};
void CustomContainer1::updateData(int index, ScreenId screen_id) {

	TEXTS textIndex{};
	if(screen_id == ScreenId::kFRONT) {
		if(textIndex < front_page_text_size) {
	       textIndex = front_page_text_indx[index];
		}
		else {
			textIndex = T_BATT_VOLT;
		}
	}
	else if(screen_id == ScreenId::kCELL_VOLTAGE) {
		if(textIndex < cell_voltage_page_text_size) {
	       textIndex = cell_voltage_page_text_indx[index];
		}
		else {
			textIndex = T_BATT_VOLT;
		}
	}
	else if(screen_id == ScreenId::kTEMPERATURE) {
		if(textIndex < temperature_page_text_size) {
	       textIndex = temperature_page_text_indx[index];
		}
		else {
			textIndex = T_BATT_VOLT;
		}
	}
	Label.setTypedText(TypedText(textIndex));

	switch(textIndex) {
	case TEXTS::T_REMAINING_CAPACITY:
		unit.setTypedText(TEXTS::T_CAPACITY_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE,data_screen1_copy.remaining_capacity);
#endif
		break;
	case TEXTS::T_PACK_CAPACITY:
		unit.setTypedText(TEXTS::T_CAPACITY_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.pack_capacity);
#endif
		break;
	case TEXTS::T_MOS_TEMP:
		unit.setTypedText(TEXTS::T_TEMP_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.mos_temp);
#endif
		break;
	case TEXTS::T_AMB_TEMP:
		unit.setTypedText(TEXTS::T_TEMP_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.ambient_temp);
#endif
		break;
	case TEXTS::T_CELL_MAX_TEMP:
		unit.setTypedText(TEXTS::T_TEMP_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.cell_temp_max);
#endif
		break;
	case TEXTS::T_CELL_MIN_TEMP:
		unit.setTypedText(TEXTS::T_TEMP_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.cell_temp_min);
#endif
		break;
	case TEXTS::T_CELL_MIN_VOLT:
		unit.setTypedText(TEXTS::T_CELL_VOLT_UNIT);
#ifndef SIMULATOR
		//ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.cell_volt_min);
		Unicode::snprintf(label_valueBuffer, LABEL_VALUE_SIZE, "%d", static_cast<uint32_t>(data_screen1_copy.cell_volt_min));
#endif
		break;
	case TEXTS::T_CELL_MAX_VOLT:
		unit.setTypedText(TEXTS::T_CELL_VOLT_UNIT);
#ifndef SIMULATOR
		//ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.cell_volt_max);
		Unicode::snprintf(label_valueBuffer, LABEL_VALUE_SIZE, "%d", static_cast<uint32_t>(data_screen1_copy.cell_volt_max));
#endif
		break;
	case TEXTS::T_BATT_CURRENT:
		unit.setTypedText(TEXTS::T_CURRENT_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.batt_current);
#endif
		break;
	case TEXTS::T_BATT_VOLT:
		unit.setTypedText(TEXTS::T_VOLTAGE_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.batt_volt);
#endif
		break;
	case TEXTS::T_SOC:
		unit.setTypedText(TEXTS::T_SOC_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.soc);
#endif
		break;
	case TEXTS::T_CELL_VOLT_1:
	case TEXTS::T_CELL_VOLT_2:
	case TEXTS::T_CELL_VOLT_3:
	case TEXTS::T_CELL_VOLT_4:
	case TEXTS::T_CELL_VOLT_5:
	case TEXTS::T_CELL_VOLT_6:
	case TEXTS::T_CELL_VOLT_7:
	case TEXTS::T_CELL_VOLT_8:
	case TEXTS::T_CELL_VOLT_9:
	case TEXTS::T_CELL_VOLT_10:
	case TEXTS::T_CELL_VOLT_11:
	case TEXTS::T_CELL_VOLT_12:
	case TEXTS::T_CELL_VOLT_13:
	case TEXTS::T_CELL_VOLT_14:
	case TEXTS::T_CELL_VOLT_15:
	case TEXTS::T_CELL_VOLT_16:
		unit.setTypedText(TEXTS::T_CELL_VOLT_UNIT);
#ifndef SIMULATOR
		Unicode::snprintf(label_valueBuffer, LABEL_VALUE_SIZE, "%d", static_cast<uint32_t>(data_screen1_copy.cell_volt[index]));
#endif
		break;
	case TEXTS::T_TEMP_1:
	case TEXTS::T_TEMP_2:
	case TEXTS::T_TEMP_3:
	case TEXTS::T_TEMP_4:
		unit.setTypedText(TEXTS::T_TEMP_UNIT);
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.cell_temp[index%4]);
#endif
		break;
	default:
#ifndef SIMULATOR
		ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, data_screen1_copy.batt_volt);
#endif
		break;
	}
	label_value.invalidate();


}
