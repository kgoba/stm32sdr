TARGET          = binary
LDSCRIPT        = stm32f446re.ld

OPENCM3_DIR     = ./libopencm3
CMSIS_DIR       = ./CMSIS/CMSIS

OBJS            += main.o coefs.o

ARCH_FLAGS      += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDLIBS          += -lopencm3_stm32f4
DEFINES         += -DSTM32F4

#INCLUDES        += -I$(CMSIS_DIR)/Core/Include
#INCLUDES        += -I$(CMSIS_DIR)/DSP/Include
INCLUDES        += -I$(CMSIS_DIR)/Include
LDFLAGS         += -L$(CMSIS_DIR)/Lib/GCC
DEFINES         += -DARM_MATH_CM4 -D__FPU_PRESENT=1
LDLIBS          += -larm_cortexM4lf_math

INCLUDES        += -I$(OPENCM3_DIR)/include
LDFLAGS         += -L$(OPENCM3_DIR)/lib

CFLAGS          += -Os -ggdb3 -ffunction-sections -fdata-sections $(INCLUDES) $(DEFINES)
CXXFLAGS        += -Os -ggdb3 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions $(INCLUDES) $(DEFINES)
CPPFLAGS        += -MD
LDFLAGS         += -static -nostartfiles -Wl,--gc-sections
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group


#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: $(TARGET).bin $(TARGET).elf $(TARGET).lst
	@arm-none-eabi-size $(TARGET).elf

$(TARGET).lst: $(TARGET).elf
	@arm-none-eabi-objdump -S $< >$@

clean:
	$(Q)$(RM) -rf $(TARGET).* *.o

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
