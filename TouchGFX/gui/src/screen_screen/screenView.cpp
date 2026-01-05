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
/*
void screenView::increment_counter() {
    static float counter = 0;
    counter += 0.1;
	Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.2f",counter);
	textArea1.invalidate();
}
*/
void  screenView::process_uart(volatile serialData_t& data) {
	//memcpy(static_cast<void*>(&data_screen1_copy),  const_cast<const serialData_t*>(&data), sizeof(serialData_t));
	/*for(int i=0; i<6; i++) {
		for(int j=0; j<15; i++) {
		scrollList1ListItems[i].updateData(j);
		}
	}*/

	/*for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area
*/
Unicode::snprintfFloat(textArea2Buffer, TEXTAREA2_SIZE, "%.2f",data.batt_volt);
    textArea2.invalidate();
}

