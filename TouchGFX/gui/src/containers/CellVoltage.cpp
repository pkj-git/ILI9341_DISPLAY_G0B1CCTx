#include <gui/containers/CellVoltage.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#include <cmath>
#include <cstdio>
#include<gui/common/floatPrint.hpp>
#include <array>

void CellVoltage::updateData(int16_t itemIndex, volatile serialData_t& data) {
    if (itemIndex < 0 || itemIndex >= 16) {
        formatFloat2(labelValueBuffer, LABELVALUE_SIZE, 0.0f);
        labelValue.invalidate();
        return;
    }

    static const std::array<TEXTS, 16> cellTextKeys = {
        TEXTS::T_CELL1_VOLT, TEXTS::T_CELL2_VOLT, TEXTS::T_CELL3_VOLT, TEXTS::T_CELL4_VOLT,
        TEXTS::T_CELL5_VOLT, TEXTS::T_CELL6_VOLT, TEXTS::T_CELL7_VOLT, TEXTS::T_CELL8_VOLT,
        TEXTS::T_CELL9_VOLT, TEXTS::T_CELL10_VOLT, TEXTS::T_CELL11VOLT, TEXTS::T_CELL12_VOLT,
        TEXTS::T_CELL13_VOLT, TEXTS::T_CELL14_VOLT, TEXTS::T_CELL15_VOLT, TEXTS::T_CELL16_VOLT
    };

    Label.setTypedText(TypedText(cellTextKeys[itemIndex]));
    float value = data.cell_volt[itemIndex];
    formatFloat2(labelValueBuffer, LABELVALUE_SIZE, value);
    labelValue.invalidate();
}


CellVoltage::CellVoltage()
{

}

void CellVoltage::initialize()
{
    CellVoltageBase::initialize();
}
