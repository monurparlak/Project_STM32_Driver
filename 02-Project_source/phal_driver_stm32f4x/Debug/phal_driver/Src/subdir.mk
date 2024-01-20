################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../phal_driver/Src/phal_gpio_stm32f4x.c \
../phal_driver/Src/phal_spi_stm32f4x.c 

OBJS += \
./phal_driver/Src/phal_gpio_stm32f4x.o \
./phal_driver/Src/phal_spi_stm32f4x.o 

C_DEPS += \
./phal_driver/Src/phal_gpio_stm32f4x.d \
./phal_driver/Src/phal_spi_stm32f4x.d 


# Each subdirectory must supply rules for building sources it contributes
phal_driver/Src/%.o phal_driver/Src/%.su phal_driver/Src/%.cyclo: ../phal_driver/Src/%.c phal_driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/my_computer/Project_STM32_Driver/02-Project_source/phal_driver_stm32f4x/phal_driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-phal_driver-2f-Src

clean-phal_driver-2f-Src:
	-$(RM) ./phal_driver/Src/phal_gpio_stm32f4x.cyclo ./phal_driver/Src/phal_gpio_stm32f4x.d ./phal_driver/Src/phal_gpio_stm32f4x.o ./phal_driver/Src/phal_gpio_stm32f4x.su ./phal_driver/Src/phal_spi_stm32f4x.cyclo ./phal_driver/Src/phal_spi_stm32f4x.d ./phal_driver/Src/phal_spi_stm32f4x.o ./phal_driver/Src/phal_spi_stm32f4x.su

.PHONY: clean-phal_driver-2f-Src

