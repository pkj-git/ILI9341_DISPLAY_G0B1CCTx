/*
 * SprintFloat.cpp
 *
 *  Created on: Dec 27, 2025
 *      Author: jainp
 */

#include <gui/common/SprintFloat.hpp>

void ConvertToFloatString(touchgfx::Unicode::UnicodeChar* buffer, const uint16_t size, float val) {
    int32_t scaled = static_cast<int32_t>(val * 100.0f);  // fixed-point
    int32_t before = scaled / 100;
    int32_t after  = scaled % 100;

    touchgfx::Unicode::snprintf(buffer, size, "%d.%02d", before, after);
}


