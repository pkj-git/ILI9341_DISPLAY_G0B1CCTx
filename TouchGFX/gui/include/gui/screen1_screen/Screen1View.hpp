#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#include <serial_data.h>
class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) override;
    virtual void process_uart(volatile serialData_t& data);

protected:
};

#endif // SCREEN1VIEW_HPP
