#ifndef SETTABLE_HPP
#define SETTABLE_HPP

#include <array>
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
    void updateData(int16_t itemIndex, const volatile std::array<float, 4U>& data);
protected:
};

#endif // SETTABLE_HPP
