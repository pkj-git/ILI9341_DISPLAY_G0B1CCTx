#ifndef PAGE1_HPP
#define PAGE1_HPP

#include <gui_generated/containers/Page1Base.hpp>
#include "serial_data.h"

class Page1 : public Page1Base
{
public:
	enum class display : uint8_t {
		Batt_volt = 0,
		batt_curr,
		cell_max,
		cell_min,
		cell_max_temp,
		cell_min_temp,
		amb_temp,
		mos_temp,
		pack_cap,
		remaining_cap,
		soc,
		soh
	};
    Page1();
    virtual ~Page1() {}

    virtual void initialize();
    void updateData(int16_t itemIndex, volatile serialData_t& data);

protected:
};

#endif // PAGE1_HPP
