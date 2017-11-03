ULP_APP_NAME ?= ulp_$(COMPONENT_NAME)
ULP_S_SOURCES = $(addprefix $(COMPONENT_PATH)/ulp/, \
	main.S \
	i2c.S \
	i2c_sht3x.S \
)

ULP_EXP_DEP_OBJECTS := $(ULP_APP_NAME).o

include $(IDF_PATH)/components/ulp/component_ulp_common.mk

$(ULP_ELF): $(ULP_OBJECTS) $(ULP_LD_SCRIPT)
	$(summary) ULP_LD $(patsubst $(PWD)/%,%,$(CURDIR))/$@
	$(ULP_LD) -o $@ -A elf32-esp32ulp -Map=$(ULP_MAP) -T $(ULP_LD_SCRIPT) $(ULP_OBJECTS)

