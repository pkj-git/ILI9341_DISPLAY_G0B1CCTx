#include <gui/cellvoltage_screen/CellVoltageView.hpp>

#include <cstring>
#include <gui/containers/CellVoltage.hpp>

static volatile serialData_t data_screen1_copy;

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

void CellVoltageView::scrollList1UpdateItem(CellVoltage& item, int16_t itemIndex) {
	item.updateData(itemIndex, data_screen1_copy);
}


void CellVoltageView::process_uart(volatile serialData_t& data) {
    memcpy(const_cast<serialData_t*>(&data_screen1_copy),
           const_cast<const serialData_t*>(&data),
           sizeof(serialData_t));
}

void CellVoltageView::updateScreen() {
	for (int i = 0; i < scrollList1.getNumberOfItems(); i++)
	{
		scrollList1.itemChanged(i);
	}
	scrollList1.invalidate(); // Redraw the entire list area
}
