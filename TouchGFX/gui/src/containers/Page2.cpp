#include <gui/containers/Page2.hpp>
#include <texts\TextKeysAndLanguages.hpp>
#include <cmath>
#include <cstdio>
#include<gui/common/floatPrint.hpp>
#include<array>
Page2::Page2()
{

}

void Page2::initialize()
{
    Page2Base::initialize();
}

void Page2::updateData(int16_t itemIndex, volatile serialData_t& data) {
    // itemIndex expected to be 0..15 for Cell1..Cell16
    if (itemIndex < 0 || itemIndex >= 16) {
        formatFloat2(labelValueBuffer, LABELVALUE_SIZE, 0.0f);
        labelValue.invalidate();
        return;
    }

    static const std::array<TEXTS, 16> cellTextKeys = {
        TEXTS::T_CELL1_TEMP, TEXTS::T_CELL2_TEMP, TEXTS::T_CELL3_TEMP, TEXTS::T_CELL4_TEMP,
        TEXTS::T_CELL5_TEMP, TEXTS::T_CELL6_TEMP, TEXTS::T_CELL7_TEMP, TEXTS::T_CELL8_TEMP,
        TEXTS::T_CELL9_TEMP, TEXTS::T_CELL10_TEMP, TEXTS::T_CELL11_TEMP, TEXTS::T_CELL12_TEMP,
        TEXTS::T_CELL13_TEMP, TEXTS::T_CELL14_TEMP, TEXTS::T_CELL15_TEMP, TEXTS::T_CELL16_TEMP
    };

    // Set label text for this cell
    Label.setTypedText(TypedText(cellTextKeys[itemIndex]));

    // Format the cell voltage (or temp) value and invalidate
    float value = data.cell_volt[itemIndex];
    formatFloat2(labelValueBuffer, LABELVALUE_SIZE, value);
    labelValue.invalidate();
}
