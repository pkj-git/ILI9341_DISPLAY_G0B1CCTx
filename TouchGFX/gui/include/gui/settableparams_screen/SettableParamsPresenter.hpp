#ifndef SETTABLEPARAMSPRESENTER_HPP
#define SETTABLEPARAMSPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class SettableParamsView;

class SettableParamsPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    SettableParamsPresenter(SettableParamsView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~SettableParamsPresenter() {}
    void process_uart(volatile serialData_t& data) override;

private:
    SettableParamsPresenter();

    SettableParamsView& view;
};

#endif // SETTABLEPARAMSPRESENTER_HPP
