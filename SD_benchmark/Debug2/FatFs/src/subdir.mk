################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FatFs/src/diskio.c \
../FatFs/src/ff.c \
../FatFs/src/ff_gen_drv.c 

OBJS += \
./FatFs/src/diskio.o \
./FatFs/src/ff.o \
./FatFs/src/ff_gen_drv.o 

C_DEPS += \
./FatFs/src/diskio.d \
./FatFs/src/ff.d \
./FatFs/src/ff_gen_drv.d 


# Each subdirectory must supply rules for building sources it contributes
FatFs/src/%.o: ../FatFs/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DUSE_HAL_LEGACY -DSTM32F0 -DSTM32F070RBTx -DNUCLEO_F070RB -DDEBUG -DSTM32F070xB -DUSE_HAL_DRIVER -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc/Legacy" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src/drivers" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/Utilities/STM32F0xx-Nucleo" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/uzLib" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/Utilities/STM32F0xx-Nucleo" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/inc" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/device" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/core" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


