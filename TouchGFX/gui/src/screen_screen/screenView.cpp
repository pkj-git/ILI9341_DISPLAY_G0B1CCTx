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
    static float counter = 0;
    counter += 0.1;
	Unicode::snprintfFloat(textArea1Buffer, TEXTAREA1_SIZE, "%.2f",counter);
	textArea1.invalidate();
}
