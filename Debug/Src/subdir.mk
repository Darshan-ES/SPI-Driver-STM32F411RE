################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SPI_example1.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/SPI_example1.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/SPI_example1.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -I"C:/Users/Darshan Salian/STM32CubeIDE/workspace_1.16.0/STM32F411RE_DRIVER/drivers/Src" -I"C:/Users/Darshan Salian/STM32CubeIDE/workspace_1.16.0/STM32F411RE_DRIVER/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/SPI_example1.cyclo ./Src/SPI_example1.d ./Src/SPI_example1.o ./Src/SPI_example1.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

