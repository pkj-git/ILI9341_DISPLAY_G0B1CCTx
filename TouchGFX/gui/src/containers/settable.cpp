#include <gui/containers/settable.hpp>
#include <gui/common/SprintFloat.hpp>

settable::settable()
{

}

void settable::initialize()
{
    settableBase::initialize();
	ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, value);
	label_value.invalidate();

}

void settable::increament() {
     value += 1.0f;
	 ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, value);
	 label_value.invalidate();
}

void settable::setValue(float val) {
	value = val;
	ConvertToFloatString(label_valueBuffer, LABEL_VALUE_SIZE, value);
	label_value.invalidate();
}