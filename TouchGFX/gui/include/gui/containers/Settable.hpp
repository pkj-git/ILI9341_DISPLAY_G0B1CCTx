#ifndef SETTABLE_HPP
#define SETTABLE_HPP

#include <gui_generated/containers/SettableBase.hpp>
#include "serial_data.h"

class Settable : public SettableBase
{
public:
    Settable();
    virtual ~Settable() {}

    virtual void initialize();
    virtual void incrementItem(int index);
    virtual void decrementItem(int index);
    void updateData(int16_t itemIndex, volatile serialData_t& data);
protected:
};

#endif // SETTABLE_HPP
