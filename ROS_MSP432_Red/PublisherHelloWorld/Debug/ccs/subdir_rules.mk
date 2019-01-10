################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
ccs/%.obj: ../ccs/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP432 Compiler'
	"/opt/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/pops/MicroControllers/ROS_MSP432_Red/RosMspHwdLib/src" --include_path="/home/pops/ti/RosLib" --include_path="/home/pops/ti/simplelink_msp432p4_sdk_2_20_00_12/source" --include_path="/opt/ti/ccsv8/ccs_base/arm/include" --include_path="/opt/ti/ccsv8/ccs_base/arm/include/CMSIS" --include_path="/opt/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.4.LTS/include" --advice:power="7,9,10" --define=__MSP432P401R__ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="ccs/$(basename $(<F)).d_raw" --obj_directory="ccs" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


