################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/current.cpp \
../src/lclcmd.cpp \
../src/monitorLoop.cpp \
../src/serialio.cpp \
../src/stm32Info.cpp 

S_UPPER_SRCS += \
../src/getSP.S \
../src/startup_stm32f103xb.S 

C_SRCS += \
../src/adc.c \
../src/dma.c \
../src/gpio.c \
../src/i2c.c \
../src/main.c \
../src/rtc.c \
../src/spi.c \
../src/stm32f1xx_hal_msp.c \
../src/stm32f1xx_it.c \
../src/system_stm32f1xx.c \
../src/tim.c \
../src/usart.c 

C_DEPS += \
./src/adc.d \
./src/dma.d \
./src/gpio.d \
./src/i2c.d \
./src/main.d \
./src/rtc.d \
./src/spi.d \
./src/stm32f1xx_hal_msp.d \
./src/stm32f1xx_it.d \
./src/system_stm32f1xx.d \
./src/tim.d \
./src/usart.d 

OBJS += \
./src/adc.o \
./src/current.o \
./src/dma.o \
./src/getSP.o \
./src/gpio.o \
./src/i2c.o \
./src/lclcmd.o \
./src/main.o \
./src/monitorLoop.o \
./src/rtc.o \
./src/serialio.o \
./src/spi.o \
./src/startup_stm32f103xb.o \
./src/stm32Info.o \
./src/stm32f1xx_hal_msp.o \
./src/stm32f1xx_it.o \
./src/system_stm32f1xx.o \
./src/tim.o \
./src/usart.o 

S_UPPER_DEPS += \
./src/getSP.d \
./src/startup_stm32f103xb.d 

CPP_DEPS += \
./src/current.d \
./src/lclcmd.d \
./src/monitorLoop.d \
./src/serialio.d \
./src/stm32Info.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DUSE_FULL_LL_DRIVER -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F103xB -DSTM32MON -DUSE_FULL_LL_DRIVER -DSTM32F1 -I../Inc/ -I../include/ -I../Drivers/CMSIS/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc/ -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include/ -I"C:\Development\Arduino\monitor\src" -I"C:\Program Files (x86)\GNU Tools Arm Embedded\9 2019-q4-major\arm-none-eabi\include" -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wa,-adhlns="$@.lst" --save-temps -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -x assembler-with-cpp -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F30X -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


