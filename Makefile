TARGET          = binary
LDSCRIPT        = target/stm32f446re.ld

OPENCM3_DIR     = ./libopencm3
CMSIS_DIR       = ./CMSIS/CMSIS

OBJS            += src/main.o src/periph.o src/dsp.o src/coefs.o

include target/stm32f446re.mk

#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all dirs flash

all: dirs .build/$(TARGET).bin .build/$(TARGET).elf .build/$(TARGET).lst
	@arm-none-eabi-size .build/$(TARGET).elf

dirs:
	@mkdir -p .build

.build/$(TARGET).lst: .build/$(TARGET).elf
	@arm-none-eabi-objdump -S $< >$@

flash: .build/$(TARGET).bin
	@st-flash write .build/$(TARGET).bin 0x8000000

clean:
	$(Q)$(RM) .build/$(TARGET).* src/*.o src/*.d

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
