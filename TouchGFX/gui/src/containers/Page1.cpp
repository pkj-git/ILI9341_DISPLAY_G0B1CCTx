#include <gui/containers/Page1.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#include <cmath>
#include <cstdio>
#include<gui/common/floatPrint.hpp>

Page1::Page1()
{

}


void Page1::initialize()
{
    Page1Base::initialize();
}

void Page1::updateData(int16_t itemIndex, volatile serialData_t& data) {
	if (itemIndex > 11) {
		return;
	}
	display d{static_cast<display>(itemIndex)};
    switch(d) {
        case display::Batt_volt:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.batt_volt);
            Label.setTypedText(TypedText(TEXTS::T_BATT_VOLT));
            labelValue.invalidate();
            break;
        case display::batt_curr:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.batt_current);
            Label.setTypedText(TypedText(TEXTS::T_BATT_CURRENT));
            labelValue.invalidate();
            break;
        case display::soc:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.soc);
            Label.setTypedText(TypedText(TEXTS::T_SOC));
            labelValue.invalidate();
            break;
        case display::soh:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.soh);
            Label.setTypedText(TypedText(TEXTS::T_SOH));
            labelValue.invalidate();
            break;
        case display::amb_temp:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.ambient_temp);
            Label.setTypedText(TypedText(TEXTS::T_AMB_TEMP));
            labelValue.invalidate();
            break;
        case display::cell_max:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.cell_volt_max);
            Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_VOLT));
            labelValue.invalidate();
            break;
        case display::cell_min:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.cell_volt_min);
            Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_VOLT));
            labelValue.invalidate();
            break;
        case display::cell_max_temp:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.cell_temp_max);
            Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_TEMP));
            labelValue.invalidate();
            break;
        case display::cell_min_temp:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.cell_temp_min);
            Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_TEMP));
            labelValue.invalidate();
            break;
        case display::mos_temp:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.mos_temp);
            Label.setTypedText(TypedText(TEXTS::T_MOS_TEMP));
            labelValue.invalidate();
            break;
        case display::pack_cap:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.pack_capacity);
            Label.setTypedText(TypedText(TEXTS::T_REMAINING_CAP));
            labelValue.invalidate();
            break;
        case display::remaining_cap:
            formatFloat2(labelValueBuffer, LABELVALUE_SIZE, data.remaining_capacity);
            Label.setTypedText(TypedText(TEXTS::T_REMAINING_CAP));
            labelValue.invalidate();
            break;
        default:
            break;
    }
}
