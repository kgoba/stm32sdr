TARGET          = binary
LDSCRIPT        = target/stm32f446re.ld

OPENCM3_DIR     = ./libopencm3
CMSIS_DIR       = ./CMSIS/CMSIS

OBJS            += src/main.o src/coefs.o

include target/stm32f446re.mk

#include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all dirs

all: dirs .build/$(TARGET).bin .build/$(TARGET).elf .build/$(TARGET).lst
	@arm-none-eabi-size .build/$(TARGET).elf

dirs:
	@mkdir -p .build

.build/$(TARGET).lst: .build/$(TARGET).elf
	@arm-none-eabi-objdump -S $< >$@

clean:
	$(Q)$(RM) .build/$(TARGET).* src/*.o src/*.d

#include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
