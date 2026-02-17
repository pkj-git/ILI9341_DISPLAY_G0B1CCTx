#include<gui/common/floatPrint.hpp>
#include<math.h>
void formatFloat2(touchgfx::Unicode::UnicodeChar* buf, size_t bufSize, float value)
{
    bool neg = std::signbit(value);
    float av = std::fabs(value);
    uint32_t int_part = static_cast<uint32_t>(av);
    uint32_t frac = static_cast<uint32_t>(std::floor((av - int_part) * 100.0f + 0.5f));
    if (frac >= 100) { frac = 0; int_part += 1; }
    if (neg) {
        touchgfx::Unicode::snprintf(buf, bufSize, "-%u.%02u", int_part, frac);
    } else {
    	touchgfx::Unicode::snprintf(buf, bufSize, "%u.%02u", int_part, frac);
    }
}
