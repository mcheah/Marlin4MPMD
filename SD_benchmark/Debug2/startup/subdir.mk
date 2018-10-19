################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f070xb.s 

OBJS += \
./startup/startup_stm32f070xb.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc/Legacy" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src/drivers" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/uzLib" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/Utilities/STM32F0xx-Nucleo" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/inc" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/device" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/core" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


