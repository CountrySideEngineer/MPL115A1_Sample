################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/MPL115A1.cpp 

OBJS += \
./src/MPL115A1.o 

CPP_DEPS += \
./src/MPL115A1.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -I"E:\development\SensorSamples\MPL115A1\dev\RaspberryPi\src\MPL115A1\src" -I"E:\development\SensorSamples\MPL115A1\dev\RaspberryPi\src\MPL115A1\src\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


