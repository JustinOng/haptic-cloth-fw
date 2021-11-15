
DEVICE = PFS172
F_CPU = 1000000
TARGET_VDD_MV = 5000
TARGET_VDD = 5.0

# ---------------------------------------------------------------------

OUTPUT_NAME = VibroCloth_$(DEVICE)

ARCH = pdk14

BUILD_DIR = .build
OUTPUT_DIR = .output

OUTPUT = $(OUTPUT_DIR)/$(OUTPUT_NAME)

SOURCES = main.c
OBJECTS = $(patsubst %.c,$(BUILD_DIR)/%.rel,$(SOURCES))

# http://sdcc.sourceforge.net/doc/sdccman.pdf
COMPILE = sdcc -m$(ARCH) -c --std-sdcc11 --opt-code-size -D$(DEVICE) -DF_CPU=$(F_CPU) -DTARGET_VDD_MV=$(TARGET_VDD_MV) -I/opt/sdcc/device/include -I. -Ifree-pdk-examples/include
LINK = sdcc -m$(ARCH)
EASYPDKPROG = easypdkprog

# symbolic targets:
all: size

print-%: ; @echo $* = $($*)

$(BUILD_DIR)/%.rel: %.c
	@mkdir -p $(dir $@)
	$(COMPILE) -o $@ $<

$(OUTPUT).ihx: $(OBJECTS)
	@mkdir -p $(dir $(OUTPUT))
	$(LINK) --out-fmt-ihx -o $(OUTPUT).ihx $(OBJECTS)

$(OUTPUT).bin: $(OUTPUT).ihx
	makebin -p $(OUTPUT).ihx $(OUTPUT).bin

build: $(OUTPUT).bin

size: build
	@echo '---------- Segments ----------'
	@egrep '(ABS,CON)|(REL,CON)' $(OUTPUT).map | gawk --non-decimal-data '{dec = sprintf("%d","0x" $$2); print dec " " $$0}' | /usr/bin/sort -n -k1 | cut -f2- -d' '
	@echo '------------------------------'
	@stat -L --printf "Size of $(OUTPUT_NAME).bin: %s bytes\n" $(OUTPUT).bin

flash: size
	$(EASYPDKPROG) -n $(DEVICE) write $(OUTPUT).ihx

run:
	$(EASYPDKPROG) -r $(TARGET_VDD) start

clean:
	rm -r -f $(BUILD_DIR) $(OUTPUT_DIR)