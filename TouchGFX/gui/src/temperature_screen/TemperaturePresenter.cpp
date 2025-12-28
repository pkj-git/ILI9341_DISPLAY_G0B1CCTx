#include <gui/temperature_screen/TemperatureView.hpp>
#include <gui/temperature_screen/TemperaturePresenter.hpp>

TemperaturePresenter::TemperaturePresenter(TemperatureView& v)
    : view(v)
{

}

void TemperaturePresenter::activate()
{

}

void TemperaturePresenter::deactivate()
{

}

#ifndef SIMULATOR
void TemperaturePresenter::process_uart(volatile serialData_t& data) {
	view.process_uart(data);
}
#endif
