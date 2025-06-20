################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/BH1750.c \
../Core/Inc/ILI9225.c \
../Core/Inc/dht22.c 

OBJS += \
./Core/Inc/BH1750.o \
./Core/Inc/ILI9225.o \
./Core/Inc/dht22.o 

C_DEPS += \
./Core/Inc/BH1750.d \
./Core/Inc/ILI9225.d \
./Core/Inc/dht22.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/BH1750.cyclo ./Core/Inc/BH1750.d ./Core/Inc/BH1750.o ./Core/Inc/BH1750.su ./Core/Inc/ILI9225.cyclo ./Core/Inc/ILI9225.d ./Core/Inc/ILI9225.o ./Core/Inc/ILI9225.su ./Core/Inc/dht22.cyclo ./Core/Inc/dht22.d ./Core/Inc/dht22.o ./Core/Inc/dht22.su

.PHONY: clean-Core-2f-Inc

