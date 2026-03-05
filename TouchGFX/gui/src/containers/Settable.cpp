#include <gui/containers/Settable.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#include <cmath>
#include <cstdio>
#include<gui/common/floatPrint.hpp>
#include<array>

Settable::Settable()
{

}

void Settable::initialize()
{
    SettableBase::initialize();
}

void Settable::incrementItem(int index) {

}

void Settable::decrementItem(int index) {

}

void Settable::updateData(int16_t itemIndex, volatile serialData_t& data) {
    // itemIndex expected to be 0..3 for Para1 to Param4
   if (itemIndex < 0 || itemIndex >= 4) {
        formatFloat2(labelValueBuffer, LABELVALUE_SIZE, 0.0f);
        labelValue.invalidate();
        return;
    }

    static const std::array<TEXTS, 4> paramTextKeys = {
        TEXTS::T_MAX_CHG_CV, TEXTS::T_MAX_CHG_CC, TEXTS::T_MIN_DSG_CV, TEXTS::T_MIN_DSG_CC
    };

    // Set label text for this cell
    Label.setTypedText(TypedText(paramTextKeys[itemIndex]));

    float value = data.settable_param[itemIndex];
    formatFloat2(labelValueBuffer, LABELVALUE_SIZE, value);
    labelValue.invalidate();

}
