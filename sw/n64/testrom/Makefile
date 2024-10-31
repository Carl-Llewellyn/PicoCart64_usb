all: testrom.z64
.PHONY: all

BUILD_DIR = build
include $(N64_INST)/include/n64.mk


C_SOURCES = \
	$(wildcard *.c) \
	$(wildcard ../../picocart64_shared/*.c)

C_INCLUDES = \
	-I$(ROOTDIR)/mips64-elf/include \
	-I$(BUILD_DIR) \
	-I../../picocart64_shared/include \

OBJS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

CFLAGS += $(C_INCLUDES)

testrom.z64: N64_ROM_TITLE = "PicoCart64 Test ROM"

$(BUILD_DIR)/testrom.elf: $(OBJS)

$(BUILD_DIR)/testrom.o: $(BUILD_DIR)/git_info.h

$(BUILD_DIR)/git_info.h:
	mkdir -p $(BUILD_DIR)
	echo -n "#define GIT_REV 0x" > $(BUILD_DIR)/git_info.h
	git rev-parse --short=8 HEAD >> $(BUILD_DIR)/git_info.h
.PHONY: $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) *.z64
.PHONY: clean

-include $(wildcard $(BUILD_DIR)/*.d))