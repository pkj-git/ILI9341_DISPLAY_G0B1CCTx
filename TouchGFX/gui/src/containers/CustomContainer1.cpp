#include <gui/containers/CustomContainer1.hpp>
#include <texts\TextKeysAndLanguages.hpp>

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
		break;
	case TEXTS::T_PACK_CAPACITY:
		Label.setTypedText(TypedText(TEXTS::T_PACK_CAPACITY));
		break;
	case TEXTS::T_MOS_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_MOS_TEMP));
		break;
	case TEXTS::T_AMB_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_AMB_TEMP));
		break;
	case TEXTS::T_CELL_MAX_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_TEMP));
		break;
	case TEXTS::T_CELL_MIN_TEMP:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_TEMP));
		break;
	case TEXTS::T_CELL_MIN_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MIN_VOLT));
		break;
	case TEXTS::T_CELL_MAX_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_CELL_MAX_VOLT));
		break;
	case TEXTS::T_BATT_CURRENT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_CURRENT));
		break;
	case TEXTS::T_BATT_VOLT:
		Label.setTypedText(TypedText(TEXTS::T_BATT_VOLT));
		break;
	default:
		break;
	}

}
