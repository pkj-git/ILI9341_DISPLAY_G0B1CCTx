################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.cpp \
../TouchGFX/gui/src/settableparams_screen/SettableParamsView.cpp 

OBJS += \
./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.o \
./TouchGFX/gui/src/settableparams_screen/SettableParamsView.o 

CPP_DEPS += \
./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.d \
./TouchGFX/gui/src/settableparams_screen/SettableParamsView.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/gui/src/settableparams_screen/%.o TouchGFX/gui/src/settableparams_screen/%.su TouchGFX/gui/src/settableparams_screen/%.cyclo: ../TouchGFX/gui/src/settableparams_screen/%.cpp TouchGFX/gui/src/settableparams_screen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TouchGFX-2f-gui-2f-src-2f-settableparams_screen

clean-TouchGFX-2f-gui-2f-src-2f-settableparams_screen:
	-$(RM) ./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.cyclo ./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.d ./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.o ./TouchGFX/gui/src/settableparams_screen/SettableParamsPresenter.su ./TouchGFX/gui/src/settableparams_screen/SettableParamsView.cyclo ./TouchGFX/gui/src/settableparams_screen/SettableParamsView.d ./TouchGFX/gui/src/settableparams_screen/SettableParamsView.o ./TouchGFX/gui/src/settableparams_screen/SettableParamsView.su

.PHONY: clean-TouchGFX-2f-gui-2f-src-2f-settableparams_screen

