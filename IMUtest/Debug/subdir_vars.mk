################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../MSP_EXP432P401R_TIRTOS.cmd 

SYSCFG_SRCS += \
../i2cbmi160.syscfg 

C_SRCS += \
../bmi160_support.c \
../i2cbmi160.c \
./syscfg/ti_drivers_config.c \
../main_tirtos.c 

GEN_FILES += \
./syscfg/ti_drivers_config.c 

GEN_MISC_DIRS += \
./syscfg/ 

C_DEPS += \
./bmi160_support.d \
./i2cbmi160.d \
./syscfg/ti_drivers_config.d \
./main_tirtos.d 

OBJS += \
./bmi160_support.obj \
./i2cbmi160.obj \
./syscfg/ti_drivers_config.obj \
./main_tirtos.obj 

GEN_MISC_FILES += \
./syscfg/ti_drivers_config.h \
./syscfg/syscfg_c.rov.xs 

GEN_MISC_DIRS__QUOTED += \
"syscfg\" 

OBJS__QUOTED += \
"bmi160_support.obj" \
"i2cbmi160.obj" \
"syscfg\ti_drivers_config.obj" \
"main_tirtos.obj" 

GEN_MISC_FILES__QUOTED += \
"syscfg\ti_drivers_config.h" \
"syscfg\syscfg_c.rov.xs" 

C_DEPS__QUOTED += \
"bmi160_support.d" \
"i2cbmi160.d" \
"syscfg\ti_drivers_config.d" \
"main_tirtos.d" 

GEN_FILES__QUOTED += \
"syscfg\ti_drivers_config.c" 

C_SRCS__QUOTED += \
"../bmi160_support.c" \
"../i2cbmi160.c" \
"./syscfg/ti_drivers_config.c" \
"../main_tirtos.c" 

SYSCFG_SRCS__QUOTED += \
"../i2cbmi160.syscfg" 


