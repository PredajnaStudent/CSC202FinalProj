################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ticlang/%.o: ../ticlang/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccstheia140/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/npredajna/OneDrive - Monroe Community College/CSC202FinalProjGit/Final Project" -I"C:/Users/npredajna/OneDrive - Monroe Community College/CSC202FinalProjGit/Final Project/Debug" -I"C:/ti/mspm0_sdk_2_00_01_00/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_00_01_00/source" -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"ticlang/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


