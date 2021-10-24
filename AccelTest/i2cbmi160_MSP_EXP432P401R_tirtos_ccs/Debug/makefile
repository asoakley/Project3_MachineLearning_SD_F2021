################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS

GEN_OPTS__FLAG := --cmd_file="C:/Users/alecb/workspace_v10/tirtos_builds_MSP_EXP432P401R_release_ccs/Debug/configPkg/compiler.opt" 
GEN_CMDS__FLAG := -l"C:/Users/alecb/workspace_v10/tirtos_builds_MSP_EXP432P401R_release_ccs/Debug/configPkg/linker.cmd" 

ORDERED_OBJS += \
"./bmi160_support.obj" \
"./i2cbmi160.obj" \
"./syscfg/ti_drivers_config.obj" \
"./main_tirtos.obj" \
"../MSP_EXP432P401R_TIRTOS.cmd" \
-l"C:/Users/alecb/Downloads/sail_1_50_00_00__all/sail_1_50_00_00/source/ti/sail/lib/ccs/msp432/sail.lib" \
-l"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/display/lib/display.aem4f" \
-l"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/drivers/lib/drivers_msp432p401x.aem4f" \
-l"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/third_party/fatfs/lib/ccs/m4f/fatfs.a" \
-l"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/kernel/tirtos/packages/ti/dpl/lib/dpl_msp432p401x.aem4f" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
i2cbmi160_MSP_EXP432P401R_tirtos_ccs.out \

EXE_OUTPUTS__QUOTED += \
"i2cbmi160_MSP_EXP432P401R_tirtos_ccs.out" \

BIN_OUTPUTS += \
i2cbmi160_MSP_EXP432P401R_tirtos_ccs.hex \

BIN_OUTPUTS__QUOTED += \
"i2cbmi160_MSP_EXP432P401R_tirtos_ccs.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "i2cbmi160_MSP_EXP432P401R_tirtos_ccs.out"

dependents:
	-cd C:/Users/alecb/workspace_v10/tirtos_builds_MSP_EXP432P401R_release_ccs/Debug && $(MAKE) all

# Tool invocations
i2cbmi160_MSP_EXP432P401R_tirtos_ccs.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) C:/Users/alecb/workspace_v10/tirtos_builds_MSP_EXP432P401R_release_ccs/Debug/tirtos_builds_MSP_EXP432P401R_release_ccs.lib
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --advice:power=none --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"i2cbmi160_MSP_EXP432P401R_tirtos_ccs.map" -i"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/lib" --diag_wrap=off --display_error_number --warn_sections --xml_link_info="i2cbmi160_MSP_EXP432P401R_tirtos_ccs_linkInfo.xml" --rom_model -o "i2cbmi160_MSP_EXP432P401R_tirtos_ccs.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

i2cbmi160_MSP_EXP432P401R_tirtos_ccs.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armhex" --diag_wrap=off -o "i2cbmi160_MSP_EXP432P401R_tirtos_ccs.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "bmi160_support.obj" "i2cbmi160.obj" "syscfg\ti_drivers_config.obj" "main_tirtos.obj" 
	-$(RM) "bmi160_support.d" "i2cbmi160.d" "syscfg\ti_drivers_config.d" "main_tirtos.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
C:/Users/alecb/workspace_v10/tirtos_builds_MSP_EXP432P401R_release_ccs/Debug/tirtos_builds_MSP_EXP432P401R_release_ccs.lib:

-include ../makefile.targets

