#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>
#include "serial_data.h"

class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
 //   void increment_counter() override;
    virtual void process_uart(volatile serialData_t& data);
    void scrollList1UpdateItem(Page1& item, int16_t itemIndex) override;
    void updateScreen() override;
    bool init{false};
protected:
private:
};

#endif // SCREENVIEW_HPP
