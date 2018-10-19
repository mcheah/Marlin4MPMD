################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../uzLib/adler32.c \
../uzLib/crc32.c \
../uzLib/defl_static.c \
../uzLib/genlz77.c \
../uzLib/tgunzip.c \
../uzLib/tinfgzip.c \
../uzLib/tinflate.c \
../uzLib/tinfzlib.c 

OBJS += \
./uzLib/adler32.o \
./uzLib/crc32.o \
./uzLib/defl_static.o \
./uzLib/genlz77.o \
./uzLib/tgunzip.o \
./uzLib/tinfgzip.o \
./uzLib/tinflate.o \
./uzLib/tinfzlib.o 

C_DEPS += \
./uzLib/adler32.d \
./uzLib/crc32.d \
./uzLib/defl_static.d \
./uzLib/genlz77.d \
./uzLib/tgunzip.d \
./uzLib/tinfgzip.d \
./uzLib/tinflate.d \
./uzLib/tinfzlib.d 


# Each subdirectory must supply rules for building sources it contributes
uzLib/%.o: ../uzLib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DSTM32 -DUSE_HAL_LEGACY -DSTM32F0 -DSTM32F070RBTx -DNUCLEO_F070RB -DDEBUG -DSTM32F070xB -DUSE_HAL_DRIVER -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc/Legacy" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src/drivers" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs/src" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/FatFs" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/Utilities/STM32F0xx-Nucleo" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/uzLib" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/Utilities/STM32F0xx-Nucleo" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/inc" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/device" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/CMSIS/core" -I"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


