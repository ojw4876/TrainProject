################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/st25r200/rfal_rfst25r200.c \
../Drivers/BSP/Components/st25r200/st25r200.c \
../Drivers/BSP/Components/st25r200/st25r200_com.c \
../Drivers/BSP/Components/st25r200/st25r200_irq.c \
../Drivers/BSP/Components/st25r200/timer.c 

OBJS += \
./Drivers/BSP/Components/st25r200/rfal_rfst25r200.o \
./Drivers/BSP/Components/st25r200/st25r200.o \
./Drivers/BSP/Components/st25r200/st25r200_com.o \
./Drivers/BSP/Components/st25r200/st25r200_irq.o \
./Drivers/BSP/Components/st25r200/timer.o 

C_DEPS += \
./Drivers/BSP/Components/st25r200/rfal_rfst25r200.d \
./Drivers/BSP/Components/st25r200/st25r200.d \
./Drivers/BSP/Components/st25r200/st25r200_com.d \
./Drivers/BSP/Components/st25r200/st25r200_irq.d \
./Drivers/BSP/Components/st25r200/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/st25r200/%.o Drivers/BSP/Components/st25r200/%.su Drivers/BSP/Components/st25r200/%.cyclo: ../Drivers/BSP/Components/st25r200/%.c Drivers/BSP/Components/st25r200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DST25R200 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../X-CUBE-NFC9/Target -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/NFC09A1 -I../Drivers/BSP/Components/st25r200 -I../Middlewares/ST/rfal/Inc -I../Middlewares/ST/rfal/Src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-st25r200

clean-Drivers-2f-BSP-2f-Components-2f-st25r200:
	-$(RM) ./Drivers/BSP/Components/st25r200/rfal_rfst25r200.cyclo ./Drivers/BSP/Components/st25r200/rfal_rfst25r200.d ./Drivers/BSP/Components/st25r200/rfal_rfst25r200.o ./Drivers/BSP/Components/st25r200/rfal_rfst25r200.su ./Drivers/BSP/Components/st25r200/st25r200.cyclo ./Drivers/BSP/Components/st25r200/st25r200.d ./Drivers/BSP/Components/st25r200/st25r200.o ./Drivers/BSP/Components/st25r200/st25r200.su ./Drivers/BSP/Components/st25r200/st25r200_com.cyclo ./Drivers/BSP/Components/st25r200/st25r200_com.d ./Drivers/BSP/Components/st25r200/st25r200_com.o ./Drivers/BSP/Components/st25r200/st25r200_com.su ./Drivers/BSP/Components/st25r200/st25r200_irq.cyclo ./Drivers/BSP/Components/st25r200/st25r200_irq.d ./Drivers/BSP/Components/st25r200/st25r200_irq.o ./Drivers/BSP/Components/st25r200/st25r200_irq.su ./Drivers/BSP/Components/st25r200/timer.cyclo ./Drivers/BSP/Components/st25r200/timer.d ./Drivers/BSP/Components/st25r200/timer.o ./Drivers/BSP/Components/st25r200/timer.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-st25r200

