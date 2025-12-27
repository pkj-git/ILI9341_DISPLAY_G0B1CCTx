#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>
extern "C" {
#include "serial_data.h"
}
class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }
    virtual void process_uart(volatile serialData_t& data) {};
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
