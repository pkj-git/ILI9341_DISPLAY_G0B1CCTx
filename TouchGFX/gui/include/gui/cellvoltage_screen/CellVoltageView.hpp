#ifndef CELLVOLTAGEVIEW_HPP
#define CELLVOLTAGEVIEW_HPP

#include <gui_generated/cellvoltage_screen/CellVoltageViewBase.hpp>
#include <gui/cellvoltage_screen/CellVoltagePresenter.hpp>

class CellVoltageView : public CellVoltageViewBase
{
public:
    CellVoltageView();
    virtual ~CellVoltageView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void process_uart(volatile serialData_t& data);
    void updateScreen();
    void scrollList1UpdateItem(CellVoltage& item, int16_t itemIndex) override;
protected:
};

#endif // CELLVOLTAGEVIEW_HPP
