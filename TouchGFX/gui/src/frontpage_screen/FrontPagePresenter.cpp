#include <gui/frontpage_screen/FrontPageView.hpp>
#include <gui/frontpage_screen/FrontPagePresenter.hpp>

FrontPagePresenter::FrontPagePresenter(FrontPageView& v)
    : view(v)
{

}

void FrontPagePresenter::activate()
{

}

void FrontPagePresenter::deactivate()
{

}
#ifndef SIMULATOR
void FrontPagePresenter::process_uart(volatile serialData_t& data) {
	view.process_uart(data);
}
#endif
