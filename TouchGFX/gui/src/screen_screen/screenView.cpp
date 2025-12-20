#include <gui/screen_screen/screenView.hpp>

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

void screenView::increment_counter() {
    static uint32_t counter = 0;
    counter++;
	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%d",counter);
	textArea1.invalidate();
}
