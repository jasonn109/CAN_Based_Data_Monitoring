################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/candriver.c \
../Src/fonts.c \
../Src/i2c_dma.c \
../Src/main.c \
../Src/rcc.c \
../Src/ssd1306.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/uart.c 

OBJS += \
./Src/candriver.o \
./Src/fonts.o \
./Src/i2c_dma.o \
./Src/main.o \
./Src/rcc.o \
./Src/ssd1306.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/uart.o 

C_DEPS += \
./Src/candriver.d \
./Src/fonts.d \
./Src/i2c_dma.d \
./Src/main.d \
./Src/rcc.d \
./Src/ssd1306.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DSTM32F446xx -c -I../Inc -I"E:/D drive/setup_F446/F446re_candriver_I2C_DMA_MOD/CMSIS/Device/ST/STM32F4xx/Include" -I"E:/D drive/setup_F446/F446re_candriver_I2C_DMA_MOD/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/candriver.cyclo ./Src/candriver.d ./Src/candriver.o ./Src/candriver.su ./Src/fonts.cyclo ./Src/fonts.d ./Src/fonts.o ./Src/fonts.su ./Src/i2c_dma.cyclo ./Src/i2c_dma.d ./Src/i2c_dma.o ./Src/i2c_dma.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/rcc.cyclo ./Src/rcc.d ./Src/rcc.o ./Src/rcc.su ./Src/ssd1306.cyclo ./Src/ssd1306.d ./Src/ssd1306.o ./Src/ssd1306.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/uart.cyclo ./Src/uart.d ./Src/uart.o ./Src/uart.su

.PHONY: clean-Src

