#ifndef SETTABLEPARAMSVIEW_HPP
#define SETTABLEPARAMSVIEW_HPP

#include <gui_generated/settableparams_screen/SettableParamsViewBase.hpp>
#include <gui/settableparams_screen/SettableParamsPresenter.hpp>
#include <serial_data.h>
class SettableParamsView : public SettableParamsViewBase
{
public:
    SettableParamsView();
    virtual ~SettableParamsView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void process_uart(volatile serialData_t& data);
    void scrollWheel1UpdateItem(Settable& item, int16_t itemIndex) override;
    void updateScreen();// override;
protected:
};

#endif // SETTABLEPARAMSVIEW_HPP
