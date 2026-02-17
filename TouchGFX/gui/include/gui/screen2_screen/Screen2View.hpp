#ifndef SCREEN2VIEW_HPP
#define SCREEN2VIEW_HPP

#include <gui_generated/screen2_screen/Screen2ViewBase.hpp>
#include <gui/screen2_screen/Screen2Presenter.hpp>

class Screen2View : public Screen2ViewBase
{
public:
    Screen2View();
    virtual ~Screen2View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void process_uart(volatile serialData_t& data);
    void scrollList1UpdateItem(Page2& item, int16_t itemIndex) override;
    void updateScreen() override;

protected:
};

#endif // SCREEN2VIEW_HPP
