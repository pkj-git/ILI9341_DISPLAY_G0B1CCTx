#ifndef CELLVOLTAGEPRESENTER_HPP
#define CELLVOLTAGEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class CellVoltageView;

class CellVoltagePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    CellVoltagePresenter(CellVoltageView& v);

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

    void process_uart(volatile serialData_t& data) override;

    virtual ~CellVoltagePresenter() {}

private:
    CellVoltagePresenter();

    CellVoltageView& view;
};

#endif // CELLVOLTAGEPRESENTER_HPP
