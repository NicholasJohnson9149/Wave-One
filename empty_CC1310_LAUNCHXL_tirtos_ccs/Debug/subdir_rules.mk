################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
CC1310_LAUNCHXL.obj: ../CC1310_LAUNCHXL.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/Users/njohnson/Documents/SinWaves/Wave-One/empty_CC1310_LAUNCHXL_tirtos_ccs" --include_path="/Applications/ti/simplelink_cc13x0_sdk_1_30_00_06/kernel/tirtos/packages/ti/sysbios/posix" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/include" --define=DEVICE_FAMILY=cc13x0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="CC1310_LAUNCHXL.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ccfg.obj: ../ccfg.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/Users/njohnson/Documents/SinWaves/Wave-One/empty_CC1310_LAUNCHXL_tirtos_ccs" --include_path="/Applications/ti/simplelink_cc13x0_sdk_1_30_00_06/kernel/tirtos/packages/ti/sysbios/posix" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/include" --define=DEVICE_FAMILY=cc13x0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ccfg.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

empty.obj: ../empty.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/Users/njohnson/Documents/SinWaves/Wave-One/empty_CC1310_LAUNCHXL_tirtos_ccs" --include_path="/Applications/ti/simplelink_cc13x0_sdk_1_30_00_06/kernel/tirtos/packages/ti/sysbios/posix" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/include" --define=DEVICE_FAMILY=cc13x0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="empty.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main_tirtos.obj: ../main_tirtos.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="/Users/njohnson/Documents/SinWaves/Wave-One/empty_CC1310_LAUNCHXL_tirtos_ccs" --include_path="/Applications/ti/simplelink_cc13x0_sdk_1_30_00_06/kernel/tirtos/packages/ti/sysbios/posix" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-arm_17.3.0.STS/include" --define=DEVICE_FAMILY=cc13x0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="main_tirtos.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


