#include <gui/settableparams_screen/SettableParamsView.hpp>
#include<serial_data.h>
#include<gui/containers/Settable.hpp>
#include <cstring>

static std::array<float, 4U> params{};
static bool init_ = false;

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
	item.updateData(itemIndex, params);
}


void SettableParamsView::process_uart(volatile serialData_t& data) {
    if(!init_) {
    	for(int i=0; i<4; i++) {
    	    params[i] = data.settable_param[i];
    	}
    	init_ = true;
    }
}

void SettableParamsView::updateScreen() {
	for (int i = 0; i < scrollWheel1.getNumberOfItems(); i++)
	{
		//scrollWheel1.itemChanged(i);
	}
	//scrollWheel1.invalidate(); // Redraw the entire list area
}

void SettableParamsView::increment_settable_param() {
	if (!scrollWheel1.isVisible()) return;
	
    uint8_t selected_item = static_cast<uint8_t>(scrollWheel1.getSelectedItem());
	uint8_t param_index = selected_item % 4;
	
    params[param_index]++;
	
    // 1. Force the wheel to refresh the data mapping for the specific visible item
	scrollWheel1.itemChanged(param_index);

	// 2. Force a redraw of the list specifically
	scrollWheel1.invalidate();
}

void SettableParamsView::decrement_settable_param() {
	if (!scrollWheel1.isVisible()) return;
	
    volatile static  uint8_t selected_item = 0 ;
    selected_item = static_cast<uint8_t>(scrollWheel1.getSelectedItem());
    selected_item = (selected_item >= 4) ? 3 : selected_item;

	volatile uint8_t param_index = selected_item % 4;
	
    params[param_index]--;
	
    // 1. Force the wheel to refresh the data mapping for the specific visible item
	scrollWheel1.itemChanged(param_index);

	// 2. Force a redraw of the list specifically
	scrollWheel1.invalidate();
}
