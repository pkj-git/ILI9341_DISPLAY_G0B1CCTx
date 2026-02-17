#include <gui/settableparams_screen/SettableParamsView.hpp>
#include <gui/settableparams_screen/SettableParamsPresenter.hpp>

SettableParamsPresenter::SettableParamsPresenter(SettableParamsView& v)
    : view(v)
{

}

void SettableParamsPresenter::activate()
{

}

void SettableParamsPresenter::deactivate()
{

}

void SettableParamsPresenter::process_uart(volatile serialData_t& data) {
    view.process_uart(data);
}
