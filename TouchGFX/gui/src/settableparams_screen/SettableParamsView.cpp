#include <gui/settableparams_screen/SettableParamsView.hpp>
#include<serial_data.h>
#include<gui/containers/Settable.hpp>
#include <cstring>

static volatile serialData_t data_screen1_copy;


SettableParamsView::SettableParamsView()
{

}

void SettableParamsView::setupScreen()
{
    SettableParamsViewBase::setupScreen();
}

void SettableParamsView::tearDownScreen()
{
    SettableParamsViewBase::tearDownScreen();
}

void SettableParamsView::scrollWheel1UpdateItem( Settable& item, int16_t itemIndex) {
	item.updateData(itemIndex, data_screen1_copy);
}


void SettableParamsView::process_uart(volatile serialData_t& data) {
    memcpy(const_cast<serialData_t*>(&data_screen1_copy),
           const_cast<const serialData_t*>(&data),
           sizeof(serialData_t));
}

void SettableParamsView::updateScreen() {
/*	for (int i = 0; i < scrollWheel1.getNumberOfItems(); i++)
	{
		scrollWheel1.itemChanged(i);
	}
	scrollWheel1.invalidate(); // Redraw the entire list area*/
}
