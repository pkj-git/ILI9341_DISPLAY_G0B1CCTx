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
    void scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) override;
#ifndef SIMULATOR
    virtual void process_uart(volatile serialData_t& data);
#endif
protected:
};

#endif // CELLVOLTAGEVIEW_HPP
