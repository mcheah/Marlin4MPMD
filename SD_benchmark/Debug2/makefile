################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include uzLib/subdir.mk
-include startup/subdir.mk
-include src/subdir.mk
-include Utilities/STM32F0xx-Nucleo/subdir.mk
-include HAL_Driver/Src/subdir.mk
-include FatFs/src/drivers/subdir.mk
-include FatFs/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SD_benchmark.elf

# Tool invocations
SD_benchmark.elf: $(OBJS) $(USER_OBJS) C:\Users\Kuli\Documents\Git\Marlin4MPMD_LCD\SD_benchmark/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -T"C:/Users/Kuli/Documents/Git/Marlin4MPMD_LCD/SD_benchmark/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "SD_benchmark.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "SD_benchmark.elf" "SD_benchmark.bin"
	arm-none-eabi-size "SD_benchmark.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
