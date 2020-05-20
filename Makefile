DEVICE_NAME ?= Qiyun
MODULE ?= platinum
CHIP_NAME ?= CPU_K32W133G256VAxA 
CORE ?= cortex-m0
LDFILE ?= ram

DEFS += -D$(CHIP_NAME)
DEFS += -D__NO_SYSTEM_INIT
DEFS += -D__STARTUP_CLEAR_BSS -D__ATOLLIC__
DEFS += -DLDFILE=\"$(LDFILE)\"
DEVICE_PATH = ./Device/$(DEVICE_NAME)

TOOLCHAIN = arm-none-eabi
AS = $(TOOLCHAIN)-as
LD = $(TOOLCHAIN)-ld
CC = $(TOOLCHAIN)-gcc
OC = $(TOOLCHAIN)-objcopy
OD = $(TOOLCHAIN)-objdump
OS = $(TOOLCHAIN)-size
GDB = $(TOOLCHAIN)-gdb

MKDIR=mkdir -p

COMM_FLAGS += -mcpu=$(CORE) -g3 -O0 -mthumb -Wall -fmessage-length=0
ASFLAGS += $(COMM_FLAGS)

CFLAGS += $(COMM_FLAGS)
CFLAGS += -ffunction-sections 
CFLAGS += -fdata-sections
CFLAGS += -specs=nano.specs
CFLAGS += -specs=nosys.specs

TARGET = output/$(LDFILE)
LINK_FILE_PATH ?= $(DEVICE_PATH)/gcc/$(LDFILE).ld

LFLAGS += -static -nostartfiles
LFLAGS += -T$(LINK_FILE_PATH)

DEVICE_SRC += 	$(wildcard $(DEVICE_PATH)/*.c) \
				$(wildcard ./common/*.c) \
				$(wildcard ./drivers/*.c) \
				$(wildcard ./Device/drivers/*.c)

SRC += $(DEVICE_SRC)
SRC += $(wildcard Modules/$(MODULE)/*.c)


ASRC = $(wildcard $(DEVICE_PATH)/gcc/*.S)

INCLUDE += -IModules/$(MODULE)
INCLUDE += -IModules/$(MODULE)/hal
INCLUDE += -IUnity
INCLUDE += -Icommon
INCLUDE += -Idrivers
INCLUDE += -Icommon/cmsis
INCLUDE += -I$(DEVICE_PATH)
BUILD_PATH ?= build/$(TARGET)

OBJS = $(addprefix $(BUILD_PATH)/,$(addsuffix .o,$(basename $(ASRC))))
OBJS += $(addprefix $(BUILD_PATH)/,$(addsuffix .o,$(basename $(SRC))))

all: $(TARGET).elf

$(TARGET).elf: $(OBJS) 
	@echo	
	@echo Linking: $@
	@$(MKDIR) -p $(dir $@)
	$(CC) $(CFLAGS) $(LFLAGS) -o $@ $^
	$(OD) -h -S $(TARGET).elf > $(TARGET).lst
	$(OC) -O binary $(TARGET).elf $(TARGET).bin
	python ./scripts/bin2hex32.py $(TARGET).bin > $(TARGET).hex

flash: $(TARGET).elf size
	@echo
	@echo Creating .hex and .bin flash images:
	$(OC) -O ihex $< $(TARGET).hex
	$(OC) -O binary $< $(TARGET).bin
	
size: $(TARGET).elf
	@echo
	@echo == Object size ==
	@$(OS) --format=berkeley $<
	
$(BUILD_PATH)/%.o: %.c
	@echo
	@echo Compiling: $<
	@$(MKDIR) -p $(dir $@)
	$(CC) -c $(CFLAGS) $(DEFS) $(INCLUDE) -I. $< -o $@

$(BUILD_PATH)/%.o: %.S
	@echo
	@echo Assembling: $<
	@$(MKDIR) -p $(dir $@)
	$(CC) -x assembler-with-cpp -c $(ASFLAGS) $(DEFS) $< -o $@	

qemu:
	@qemu-system-arm -M ? | grep musca-b1 >/dev/null || exit
	qemu-system-arm -machine musca-b1 -cpu cortex-m33 \
	    -m 4096 -nographic -serial mon:stdio -kernel $(TARGET).elf 

gdbserver:
	qemu-system-arm -machine musca-b1 -cpu cortex-m33 \
	    -m 4096 -nographic -serial mon:stdio -kernel $(TARGET).elf -S -s 

gdb: $(TARGET).elf
	$(GDB) $^ -ex "target remote:1234"

clean: 
	@echo Cleaning:
	$(RM) -rf build output

