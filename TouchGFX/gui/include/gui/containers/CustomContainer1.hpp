#ifndef CUSTOMCONTAINER1_HPP
#define CUSTOMCONTAINER1_HPP

#include <gui_generated/containers/CustomContainer1Base.hpp>
#include <gui/common/ScreenDefs.hpp>
class CustomContainer1 : public CustomContainer1Base
{
public:
    CustomContainer1();
    virtual ~CustomContainer1() {}

    virtual void initialize();
    void updateData(int index, ScreenId screen_id);
protected:
};

#endif // CUSTOMCONTAINER1_HPP
