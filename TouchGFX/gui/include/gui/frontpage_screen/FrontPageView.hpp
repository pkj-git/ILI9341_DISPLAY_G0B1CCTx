#ifndef FRONTPAGEVIEW_HPP
#define FRONTPAGEVIEW_HPP

#include <gui_generated/frontpage_screen/FrontPageViewBase.hpp>
#include <gui/frontpage_screen/FrontPagePresenter.hpp>
#ifndef SIMULATOR
#include "serial_data.h"
#endif

class FrontPageView : public FrontPageViewBase
{
public:
    FrontPageView();
    virtual ~FrontPageView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) override;
#ifndef SIMULATOR
    virtual void process_uart(volatile serialData_t& data);
#endif
protected:
};

#endif // FRONTPAGEVIEW_HPP
