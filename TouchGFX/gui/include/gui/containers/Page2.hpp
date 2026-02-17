#ifndef PAGE2_HPP
#define PAGE2_HPP

#include <gui_generated/containers/Page2Base.hpp>
#include "serial_data.h"

class Page2 : public Page2Base
{
public:
    Page2();
    virtual ~Page2() {}

    virtual void initialize();
    void updateData(int16_t itemIndex, volatile serialData_t& data);

protected:
};

#endif // PAGE2_HPP
