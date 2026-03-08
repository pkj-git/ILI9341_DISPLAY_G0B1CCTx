################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.cpp \
../TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.cpp \
../TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.cpp \
../TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.cpp 

OBJS += \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.o \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.o \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.o \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.o 

CPP_DEPS += \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.d \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.d \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.d \
./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.d 


# Each subdirectory must supply rules for building sources it contributes
TouchGFX/generated/images/src/__generated/%.o TouchGFX/generated/images/src/__generated/%.su TouchGFX/generated/images/src/__generated/%.cyclo: ../TouchGFX/generated/images/src/__generated/%.cpp TouchGFX/generated/images/src/__generated/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m0plus -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32G0B1xx -c -I../TouchGFX/App -I../TouchGFX/target/generated -I../TouchGFX/target -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/touchgfx/framework/include -I../TouchGFX/generated/fonts/include -I../TouchGFX/generated/gui_generated/include -I../TouchGFX/generated/images/include -I../TouchGFX/generated/texts/include -I../TouchGFX/generated/videos/include -I../TouchGFX/gui/include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -femit-class-debug-always -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated:
	-$(RM) ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.cyclo ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.d ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.o ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_down_50_50_E8F6FB.svg.su ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.cyclo ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.d ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.o ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_hardware_keyboard_arrow_up_50_50_E8F6FB.svg.su ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.cyclo ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.d ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.o ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_before_50_50_E8F6FB.svg.su ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.cyclo ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.d ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.o ./TouchGFX/generated/images/src/__generated/image_icon_theme_images_image_navigate_next_50_50_E8F6FB.svg.su

.PHONY: clean-TouchGFX-2f-generated-2f-images-2f-src-2f-__generated

