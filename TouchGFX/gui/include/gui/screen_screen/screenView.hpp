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

protected:
};

#endif // SCREENVIEW_HPP
