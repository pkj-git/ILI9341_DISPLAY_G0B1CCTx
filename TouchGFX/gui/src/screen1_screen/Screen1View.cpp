#include <gui/screen1_screen/Screen1View.hpp>
#include <cstring>
serialData_t data_screen1_copy;
Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) {
	item.updateData(itemIndex);
}
void  Screen1View::process_uart(volatile serialData_t& data) {
	memcpy(&data_screen1_copy,  const_cast<const serialData_t*>(&data), sizeof(serialData_t));
	/*for(int i=0; i<6; i++) {
		for(int j=0; j<15; i++) {
		scrollList1ListItems[i].updateData(j);
		}
	}*/

	for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area

}
