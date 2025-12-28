#include <gui/frontpage_screen/FrontPageView.hpp>
#include <gui/common/ScreenDefs.hpp>

#ifndef SIMULATOR
serialData_t data_screen1_copy;
#endif
#include <cstring>
FrontPageView::FrontPageView()
{

}

void FrontPageView::setupScreen()
{
    FrontPageViewBase::setupScreen();
    scrollableContainer.setScrollbarWidth(10);
    scrollableContainer.childGeometryChanged();
}

void FrontPageView::tearDownScreen()
{
    FrontPageViewBase::tearDownScreen();
}
void FrontPageView::scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) {
	item.updateData(itemIndex, ScreenId::kFRONT);
}
#ifndef SIMULATOR
void  FrontPageView::process_uart(volatile serialData_t& data) {
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
