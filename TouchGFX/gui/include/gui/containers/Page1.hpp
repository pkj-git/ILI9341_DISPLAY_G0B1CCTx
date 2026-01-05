#ifndef PAGE1_HPP
#define PAGE1_HPP

#include <gui_generated/containers/Page1Base.hpp>
#include "serial_data.h"

class Page1 : public Page1Base
{
public:
    Page1();
    virtual ~Page1() {}

    virtual void initialize();
    void updateData(int16_t itemIndex, volatile serialData_t& data);

protected:
};

#endif // PAGE1_HPP
