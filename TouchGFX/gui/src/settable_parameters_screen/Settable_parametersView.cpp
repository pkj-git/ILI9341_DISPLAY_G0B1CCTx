#include <gui/settable_parameters_screen/Settable_parametersView.hpp>

Settable_parametersView::Settable_parametersView()
{

}

void Settable_parametersView::setupScreen()
{
    Settable_parametersViewBase::setupScreen();
}

void Settable_parametersView::tearDownScreen()
{
    Settable_parametersViewBase::tearDownScreen();
}

void Settable_parametersView::increament() {
    int selectedIndex = scrollWheel1.getSelectedItem();
    if(selectedIndex >=0 && selectedIndex <10) {    
        itemValues[selectedIndex] += 1.0f;
    } 
    scrollWheel1.itemChanged(selectedIndex);
}

void Settable_parametersView::decrement() {
    int selectedIndex = scrollWheel1.getSelectedItem();
    if(selectedIndex >=0 && selectedIndex <10) {
        itemValues[selectedIndex] -= 1.0f;
    }
    scrollWheel1.itemChanged(selectedIndex);
}

void Settable_parametersView::scrollWheel1UpdateItem(settable& item, int16_t itemIndex)
{
    item.setValue(itemValues[itemIndex]);
}
