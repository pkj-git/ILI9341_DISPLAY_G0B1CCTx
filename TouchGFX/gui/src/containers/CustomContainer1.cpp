#include <gui/containers/CustomContainer1.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#ifndef SIMULATOR
#include "serial_data.h"
extern serialData_t data_screen1_copy;
#endif

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
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_PACK_CAPACITY:
		Label.setTypedText(TypedText(TEXTS::T_PACK_CAPACITY));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_MOS_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_MOS_TEMP));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_AMB_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_AMB_TEMP));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_CELL_MAX_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_TEMP));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_CELL_MIN_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_TEMP));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_CELL_MIN_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_VOLT));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_CELL_MAX_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_VOLT));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_BATT_CURRENT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_CURRENT));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	case TEXTS::T_BATT_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_VOLT));
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	default:
#ifndef SIMULATOR
		Unicode::snprintfFloat(label_valueBuffer, LABEL_VALUE_SIZE, "%.2f",data_screen1_copy.batt_volt);
		label_value.invalidate();
#endif
		break;
	}

}
