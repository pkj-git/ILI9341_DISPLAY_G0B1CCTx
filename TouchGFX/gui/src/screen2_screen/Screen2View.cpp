#include <gui/screen2_screen/Screen2View.hpp>
#include <cstring>
static volatile serialData_t data_screen1_copy;

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
    init = true;
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::scrollList1UpdateItem(Page2& item, int16_t itemIndex) {
	item.updateData(itemIndex, data_screen1_copy);
}

void  Screen2View::process_uart(volatile serialData_t& data) {
	memcpy(const_cast<serialData_t*>(&data_screen1_copy),
	       const_cast<const serialData_t*>(&data),
	       sizeof(serialData_t));
	/*for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area*/
}


void Screen2View::updateScreen() {
	if(!init) {
		return;
	}
	for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area

}
