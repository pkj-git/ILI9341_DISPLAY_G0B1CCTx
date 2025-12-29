#ifndef SETTABLE_HPP
#define SETTABLE_HPP

#include <gui_generated/containers/settableBase.hpp>

class settable : public settableBase
{
public:
    settable();
    virtual ~settable() {}

    virtual void initialize();
    void increament();
    void decreament();
    void setValue(float val);
protected:
float value = 0.0f;
};

#endif // SETTABLE_HPP
