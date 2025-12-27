#include <gui/containers/CustomContainer1.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#include "serial_data.h"
extern serialData_t data_screen1_copy;
CustomContainer1::CustomContainer1()
{

}

void CustomContainer1::initialize()
{
    CustomContainer1Base::initialize();
}

void CustomContainer1::updateData(int index) {
	TEXTS textIndex = static_cast<TEXTS>(index);
	switch(textIndex) {
	case TEXTS::T_REMAINING_CAPACITY:
		Label.setTypedText(TypedText(TEXTS::T_REMAINING_CAPACITY));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_PACK_CAPACITY:
		Label.setTypedText(TypedText(TEXTS::T_PACK_CAPACITY));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_MOS_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_MOS_TEMP));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_AMB_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_AMB_TEMP));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_CELL_MAX_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_TEMP));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_CELL_MIN_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_TEMP));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_CELL_MIN_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_VOLT));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_CELL_MAX_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_VOLT));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_BATT_CURRENT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_CURRENT));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	case TEXTS::T_BATT_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_VOLT));
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
		break;
	default:
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.ambient_temp);
		label_value.invalidate();
		break;
	}

}
