#include <gui/screen_screen/screenView.hpp>
#include <cstring>
volatile serialData_t data_screen1_copy;
screenView::screenView()
{

}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
}
void screenView::scrollList1UpdateItem(Page1& item, int16_t itemIndex) {
	item.updateData(itemIndex, data_screen1_copy);
}

void  screenView::process_uart(volatile serialData_t& data) {
	memcpy(const_cast<serialData_t*>(&data_screen1_copy),
	       const_cast<const serialData_t*>(&data),
	       sizeof(serialData_t));
	/*for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area*/
}


void screenView::updateScreen() {
	for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area

}
