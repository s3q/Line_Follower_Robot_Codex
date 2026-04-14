################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/gcc_arm_none_eabi_9_2_1

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./avoid.o" \
"./bump.o" \
"./calibration.o" \
"./collision.o" \
"./control.o" \
"./demo.o" \
"./emergency.o" \
"./ir.o" \
"./line.o" \
"./line_follow.o" \
"./line_follow_mode.o" \
"./main.o" \
"./mission.o" \
"./mode_manager.o" \
"./motor.o" \
"./mpu6500.o" \
"./obstacle_mode.o" \
"./reaction.o" \
"./sensors.o" \
"./system_msp432p401r.o" \
"./system_test.o" \
"./ultrasonic.o" \
"./verify.o" \
"./zone_detector.o" \
"./gcc/startup_msp432p401r_gcc.o" \
-Wl,-T"../msp432p401r.lds" \
$(GEN_CMDS__FLAG) \
-l:"ti/display/lib/display.am4fg" \
-l:"ti/grlib/lib/gcc/m4f/grlib.a" \
-l:"third_party/spiffs/lib/gcc/m4f/spiffs.a" \
-l:"ti/drivers/lib/drivers_msp432p401x.am4fg" \
-l:"third_party/fatfs/lib/gcc/m4f/fatfs.a" \
-l:"ti/devices/msp432p4xx/driverlib/gcc/msp432p4xx_driverlib.a" \
-lgcc \
-lc \
-lm \
-lnosys \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include gcc/subdir_vars.mk
-include subdir_rules.mk
-include gcc/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Line_Detector_Robot.out 

EXE_OUTPUTS__QUOTED += \
"Line_Detector_Robot.out" 


# All Target
all: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "Line_Detector_Robot.out"

# Tool invocations
Line_Detector_Robot.out: $(OBJS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'GNU Linker - building target: "$@"'
	"C:/ti/gcc_arm_none_eabi_9_2_1/bin/arm-none-eabi-gcc-9.2.1.exe" -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -DDeviceFamily_MSP432P401x -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -Wall -mcpu=cortex-m4 -Wl,-Map,"Line_Detector_Robot.map" -static -Wl,--gc-sections -L"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source" -L"C:/ti/gcc_arm_none_eabi_9_2_1/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard" -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs -o"Line_Detector_Robot.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "avoid.o" "bump.o" "calibration.o" "collision.o" "control.o" "demo.o" "emergency.o" "ir.o" "line.o" "line_follow.o" "line_follow_mode.o" "main.o" "mission.o" "mode_manager.o" "motor.o" "mpu6500.o" "obstacle_mode.o" "reaction.o" "sensors.o" "system_msp432p401r.o" "system_test.o" "ultrasonic.o" "verify.o" "zone_detector.o" "gcc\startup_msp432p401r_gcc.o" 
	-$(RM) "avoid.d" "bump.d" "calibration.d" "collision.d" "control.d" "demo.d" "emergency.d" "ir.d" "line.d" "line_follow.d" "line_follow_mode.d" "main.d" "mission.d" "mode_manager.d" "motor.d" "mpu6500.d" "obstacle_mode.d" "reaction.d" "sensors.d" "system_msp432p401r.d" "system_test.d" "ultrasonic.d" "verify.d" "zone_detector.d" "gcc\startup_msp432p401r_gcc.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

