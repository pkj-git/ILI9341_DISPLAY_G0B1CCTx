#include <gui/cellvoltage_screen/CellVoltageView.hpp>
#ifndef SIMULATOR
extern serialData_t data_screen1_copy;
#endif
#include <cstring>

CellVoltageView::CellVoltageView()
{

}

void CellVoltageView::setupScreen()
{
    CellVoltageViewBase::setupScreen();
    scrollableContainer1.setScrollbarWidth(10);
    scrollableContainer1.childGeometryChanged();
}

void CellVoltageView::tearDownScreen()
{
    CellVoltageViewBase::tearDownScreen();
}

void CellVoltageView::scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) {
	item.updateData(itemIndex, ScreenId::kCELL_VOLTAGE);
}
#ifndef SIMULATOR
void  CellVoltageView::process_uart(volatile serialData_t& data) {
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
#endif
