#ifndef SETTABLE_PARAMETERSVIEW_HPP
#define SETTABLE_PARAMETERSVIEW_HPP

#include <gui_generated/settable_parameters_screen/Settable_parametersViewBase.hpp>
#include <gui/settable_parameters_screen/Settable_parametersPresenter.hpp>

class Settable_parametersView : public Settable_parametersViewBase
{
public:
    Settable_parametersView();
    virtual ~Settable_parametersView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    void  scrollWheel1UpdateItem(settable& item, int16_t itemIndex) override;
    void increament() override;
    void decrement() override;
protected:
float itemValues[10] = {0,0,0,0,0,0,0,0,0,0};

};

#endif // SETTABLE_PARAMETERSVIEW_HPP
