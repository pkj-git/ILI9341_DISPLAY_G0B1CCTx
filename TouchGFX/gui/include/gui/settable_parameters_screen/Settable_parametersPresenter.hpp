#ifndef SETTABLE_PARAMETERSPRESENTER_HPP
#define SETTABLE_PARAMETERSPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Settable_parametersView;

class Settable_parametersPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    Settable_parametersPresenter(Settable_parametersView& v);

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

    virtual ~Settable_parametersPresenter() {}

private:
    Settable_parametersPresenter();

    Settable_parametersView& view;
};

#endif // SETTABLE_PARAMETERSPRESENTER_HPP
