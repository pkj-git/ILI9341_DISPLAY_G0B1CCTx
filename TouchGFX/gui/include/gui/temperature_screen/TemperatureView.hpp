#ifndef TEMPERATUREVIEW_HPP
#define TEMPERATUREVIEW_HPP

#include <gui_generated/temperature_screen/TemperatureViewBase.hpp>
#include <gui/temperature_screen/TemperaturePresenter.hpp>

class TemperatureView : public TemperatureViewBase
{
public:
    TemperatureView();
    virtual ~TemperatureView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void scrollList1UpdateItem(CustomContainer1& item, int16_t itemIndex) override;
#ifndef SIMULATOR
    virtual void process_uart(volatile serialData_t& data);
#endif
protected:
};

#endif // TEMPERATUREVIEW_HPP
