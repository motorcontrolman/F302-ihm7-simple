################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
F302-ihm7-simple.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
F302-ihm7-simple.list \

OBJCOPY_BIN += \
F302-ihm7-simple.bin \


# All Target
all: F302-ihm7-simple.elf secondary-outputs

# Tool invocations
F302-ihm7-simple.elf: $(OBJS) $(USER_OBJS) C:\Users\r720r\STM32CubeIDE\workspace_1.3.0\F302-ihm7-simple\STM32F302R8TX_FLASH.ld
	arm-none-eabi-gcc -o "F302-ihm7-simple.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\r720r\STM32CubeIDE\workspace_1.3.0\F302-ihm7-simple\STM32F302R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="F302-ihm7-simple.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

F302-ihm7-simple.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "F302-ihm7-simple.list"
	@echo 'Finished building: $@'
	@echo ' '

F302-ihm7-simple.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "F302-ihm7-simple.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
