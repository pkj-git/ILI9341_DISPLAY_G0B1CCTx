#include <gui/cellvoltage_screen/CellVoltageView.hpp>
#include <gui/cellvoltage_screen/CellVoltagePresenter.hpp>

CellVoltagePresenter::CellVoltagePresenter(CellVoltageView& v)
    : view(v)
{

}

void CellVoltagePresenter::activate()
{

}

void CellVoltagePresenter::deactivate()
{

}
#ifndef SIMULATOR
void CellVoltagePresenter::process_uart(volatile serialData_t& data) {
	view.process_uart(data);
}
#endif
