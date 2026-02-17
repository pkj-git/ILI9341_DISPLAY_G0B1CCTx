#ifndef CELLVOLTAGE_HPP
#define CELLVOLTAGE_HPP

#include <gui_generated/containers/CellVoltageBase.hpp>
#include <serial_data.h>
class CellVoltage : public CellVoltageBase
{
public:
    CellVoltage();
    virtual ~CellVoltage() {}

    virtual void initialize();
    void updateData(int16_t itemIndex, volatile serialData_t& data);
protected:
};

#endif // CELLVOLTAGE_HPP
