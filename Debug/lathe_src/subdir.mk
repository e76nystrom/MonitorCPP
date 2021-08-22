################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Development/EclipseCPP/LatheCPP/lathe_src/i2cx.cpp \
C:/Development/EclipseCPP/LatheCPP/lathe_src/lcd.cpp \
C:/Development/EclipseCPP/LatheCPP/lathe_src/spix.cpp \
C:/Development/EclipseCPP/LatheCPP/lathe_src/stm32Info.cpp 

OBJS += \
./lathe_src/i2cx.o \
./lathe_src/lcd.o \
./lathe_src/spix.o \
./lathe_src/stm32Info.o 

CPP_DEPS += \
./lathe_src/i2cx.d \
./lathe_src/lcd.d \
./lathe_src/spix.d \
./lathe_src/stm32Info.d 


# Each subdirectory must supply rules for building sources it contributes
lathe_src/i2cx.o: C:/Development/EclipseCPP/LatheCPP/lathe_src/i2cx.cpp lathe_src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lathe_src/lcd.o: C:/Development/EclipseCPP/LatheCPP/lathe_src/lcd.cpp lathe_src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lathe_src/spix.o: C:/Development/EclipseCPP/LatheCPP/lathe_src/spix.cpp lathe_src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

lathe_src/stm32Info.o: C:/Development/EclipseCPP/LatheCPP/lathe_src/stm32Info.cpp lathe_src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


