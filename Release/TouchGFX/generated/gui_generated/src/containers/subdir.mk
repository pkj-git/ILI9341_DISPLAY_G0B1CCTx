################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.cpp \
../TouchGFX/generated/gui_generated/src/containers/Page1Base.cpp \
../TouchGFX/generated/gui_generated/src/containers/Page2Base.cpp \
../TouchGFX/generated/gui_generated/src/containers/SettableBase.cpp 

OBJS += \
./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.o \
./TouchGFX/generated/gui_generated/src/containers/Page1Base.o \
./TouchGFX/generated/gui_generated/src/containers/Page2Base.o \
./TouchGFX/generated/gui_generated/src/containers/SettableBase.o 

CPP_DEPS += \
./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.d \
./TouchGFX/generated/gui_generated/src/containers/Page1Base.d \
./TouchGFX/generated/gui_generated/src/containers/Page2Base.d \
./TouchGFX/generated/gui_generated/src/containers/SettableBase.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/gui_generated/src/containers/%.o TouchGFX/generated/gui_generated/src/containers/%.su TouchGFX/generated/gui_generated/src/containers/%.cyclo: ../TouchGFX/generated/gui_generated/src/containers/%.cpp TouchGFX/generated/gui_generated/src/containers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TouchGFX-2f-generated-2f-gui_generated-2f-src-2f-containers

clean-TouchGFX-2f-generated-2f-gui_generated-2f-src-2f-containers:
	-$(RM) ./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.cyclo ./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.d ./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.o ./TouchGFX/generated/gui_generated/src/containers/CellVoltageBase.su ./TouchGFX/generated/gui_generated/src/containers/Page1Base.cyclo ./TouchGFX/generated/gui_generated/src/containers/Page1Base.d ./TouchGFX/generated/gui_generated/src/containers/Page1Base.o ./TouchGFX/generated/gui_generated/src/containers/Page1Base.su ./TouchGFX/generated/gui_generated/src/containers/Page2Base.cyclo ./TouchGFX/generated/gui_generated/src/containers/Page2Base.d ./TouchGFX/generated/gui_generated/src/containers/Page2Base.o ./TouchGFX/generated/gui_generated/src/containers/Page2Base.su ./TouchGFX/generated/gui_generated/src/containers/SettableBase.cyclo ./TouchGFX/generated/gui_generated/src/containers/SettableBase.d ./TouchGFX/generated/gui_generated/src/containers/SettableBase.o ./TouchGFX/generated/gui_generated/src/containers/SettableBase.su

.PHONY: clean-TouchGFX-2f-generated-2f-gui_generated-2f-src-2f-containers

