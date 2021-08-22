################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Development/Arduino/monitor/src/max31856.cpp \
C:/Development/Arduino/monitor/src/max31865.cpp 

OBJS += \
./monitor/max31856.o \
./monitor/max31865.o 

CPP_DEPS += \
./monitor/max31856.d \
./monitor/max31865.d 


# Each subdirectory must supply rules for building sources it contributes
monitor/max31856.o: C:/Development/Arduino/monitor/src/max31856.cpp monitor/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

monitor/max31865.o: C:/Development/Arduino/monitor/src/max31865.cpp monitor/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


