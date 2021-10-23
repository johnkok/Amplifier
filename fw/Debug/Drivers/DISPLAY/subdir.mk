################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DISPLAY/display.c \
../Drivers/DISPLAY/tm_stm32f4_fonts.c \
../Drivers/DISPLAY/tm_stm32f4_ili9341.c \
../Drivers/DISPLAY/vu.c 

OBJS += \
./Drivers/DISPLAY/display.o \
./Drivers/DISPLAY/tm_stm32f4_fonts.o \
./Drivers/DISPLAY/tm_stm32f4_ili9341.o \
./Drivers/DISPLAY/vu.o 

C_DEPS += \
./Drivers/DISPLAY/display.d \
./Drivers/DISPLAY/tm_stm32f4_fonts.d \
./Drivers/DISPLAY/tm_stm32f4_ili9341.d \
./Drivers/DISPLAY/vu.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DISPLAY/%.o: ../Drivers/DISPLAY/%.c Drivers/DISPLAY/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

