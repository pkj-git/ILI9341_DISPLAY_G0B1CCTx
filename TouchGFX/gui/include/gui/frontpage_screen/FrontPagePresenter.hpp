#ifndef FRONTPAGEPRESENTER_HPP
#define FRONTPAGEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>
#ifndef SIMULATOR
#include "serial_data.h"
#endif

using namespace touchgfx;

class FrontPageView;

class FrontPagePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    FrontPagePresenter(FrontPageView& v);

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

    virtual ~FrontPagePresenter() {}
#ifndef SIMULATOR
    void process_uart(volatile serialData_t& data) override;
#endif

private:
    FrontPagePresenter();

    FrontPageView& view;
};

#endif // FRONTPAGEPRESENTER_HPP
