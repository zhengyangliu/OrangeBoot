#
# Common rules for makefiles for the PX4 bootloaders
#

BUILD_DIR	 = $(BUILD_DIR_ROOT)/$(TARGET_FILE_NAME)

COBJS		:= $(addprefix $(BUILD_DIR)/, $(patsubst %.c,%.o,$(SRCS)))
AOBJS		:= $(addprefix $(BUILD_DIR)/, $(patsubst %.S,%.o,$(ASRCS)))
OBJS		:= $(COBJS) $(AOBJS)
DEPS		:= $(COBJS:.o=.d)

ELF		 = $(BUILD_DIR)/$(TARGET_FILE_NAME).elf
BINARY	 = $(BUILD_DIR)/$(TARGET_FILE_NAME).bin
HEX      = $(BUILD_DIR)/$(TARGET_FILE_NAME).hex     

all:	debug $(BUILD_DIR) $(ELF) $(BINARY) $(HEX)

debug:
#	@echo SRCS=$(SRCS)
#	@echo COBJS=$(COBJS)
#	@echo ASRCS=$(ASRCS)
#	@echo AOBJS=$(AOBJS)
#	@echo SUBDIRS=$(SUBDIRS)


# Compile and generate dependency files
$(BUILD_DIR)/%.o:	%.c
	@echo Generating object $@
	$(CC) -c -MMD $(FLAGS) -o $@ $<

$(BUILD_DIR)/%.o:	%.S
	@echo Generating object $@
	$(CC) -c -MMD $(FLAGS) -o $@ $*.S

# Make the build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(ELF):		$(OBJS) $(MAKEFILE_LIST)
	$(CC) -o $@ $(OBJS) $(FLAGS)

$(BINARY):	$(ELF)
	$(OBJCOPY) -O binary $(ELF) $(BINARY)

$(HEX):	$(ELF)
	$(OBJCOPY) -O ihex $(ELF) $(HEX)

# Dependencies for .o files
-include $(DEPS)
