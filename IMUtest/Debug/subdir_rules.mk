################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs" --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs/Debug" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/third_party/CMSIS/Include" --include_path="C:/Users/alecb/Downloads/sail_1_50_00_00__all/sail_1_50_00_00/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/posix/ccs" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs/Debug/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-798333873: ../i2cbmi160.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1040/ccs/utils/sysconfig_1.9.0/sysconfig_cli.bat" -s "C:/ti/simplelink_msp432p4_sdk_3_40_01_02/.metadata/product.json" -s "C:/Users/alecb/Downloads/sail_1_50_00_00__all/sail_1_50_00_00/.metadata/product.json" -s "C:/ti/simplelink_msp432p4_sdk_3_40_01_02/.metadata/product.json" --script "C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs/i2cbmi160.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_config.c: build-798333873 ../i2cbmi160.syscfg
syscfg/ti_drivers_config.h: build-798333873
syscfg/syscfg_c.rov.xs: build-798333873
syscfg/: build-798333873

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs" --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs/Debug" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/third_party/CMSIS/Include" --include_path="C:/Users/alecb/Downloads/sail_1_50_00_00__all/sail_1_50_00_00/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source/ti/posix/ccs" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/alecb/workspace_v10/i2cbmi160_MSP_EXP432P401R_tirtos_ccs/Debug/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


