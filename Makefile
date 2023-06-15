#   some environment variables can be set:
#
#   FQBN        Fully Qualified Board Name; if not set in the environment
#               it will be assigned a value in this makefile
#
#   CLI_ARGS    Passed as arguments to arduino-cli for compilation
#               it will be assigned the default value of an empty string
#				
#	LIBRARIES	arduino libraries path
#
#
FQBN        ?= sandeepmistry:nRF5:DWM1001-DEV
LIBRARIES   ?= 
CLI_ARGS    ?= 

SRC        := $(wildcard *.cpp)
HDRS       := $(wildcard *.h)

ifneq ($(strip $(LIBRARIES)),)
	LIBARGS = --libraries $(LIBRARIES)   
else 
	LIBARGS = 
endif

OUT_PATH = 

$(info FQBN       is [${FQBN}])
$(info CLI_ARGS   is [${CLI_ARGS}])
$(info LIBRAIRIES is [${LIBRARIES}])
$(info SRC        is [${SRC}])
$(info HDRS       is [${HDRS}])

all: $(ELF) upload
.PHONY: all

clean:
	@echo "---> Cleaning the examples directory"
	@for f in $(shell ls ${MYDIR}); do echo $${f}; done
	
.SECONDEXPANSION:	# required for secondary expansions of $$*
% : examples/$$*/$$*.elf
	@echo Making $@
	
	
.PRECIOUS:
%.elf : $$*.ino $(SRC) $(HDRS)
	$(eval OUT_PATH := $(shell dirname $@))
	$(eval BASENAME := $(shell basename $@))
	@echo $(OUT_PATH)
	arduino-cli compile -b $(FQBN) $(CLI_ARGS) $(LIBARGS) -e -v $*.ino
	mv $(OUT_PATH)/build/$(subst :,.,$(FQBN))/*.elf $@
	



