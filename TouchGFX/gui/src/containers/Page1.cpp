#include <gui/containers/Page1.hpp>
#include <texts\TextKeysAndLanguages.hpp>
Page1::Page1()
{

}

void Page1::initialize()
{
    Page1Base::initialize();
}

void Page1::updateData(int16_t itemIndex, volatile serialData_t& data) {
    switch(itemIndex) {
        case 0:
            Unicode::snprintfFloat(labelValueBuffer, LABELVALUE_SIZE, "%.2f", data.batt_volt);
            labelValue.invalidate();
            break;
        case 1:
            Unicode::snprintfFloat(labelValueBuffer, LABELVALUE_SIZE, "%.2f", data.batt_current);
            labelValue.invalidate();
            break;
        case 2:
            Unicode::snprintfFloat(labelValueBuffer, LABELVALUE_SIZE, "%.2f", data.soc);
            labelValue.invalidate();
            break;
        case 3:
            Unicode::snprintfFloat(labelValueBuffer, LABELVALUE_SIZE, "%.2f", data.soh);
            labelValue.invalidate();
            break;
        default:
            break;
    }
}
