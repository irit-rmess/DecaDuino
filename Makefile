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

all: 
.PHONY: all

clean:
	@echo "---> Cleaning the examples directory"
	@for f in $(shell ls examples/); do echo examples/$${f}/ ; rm -rf examples/$${f}/build examples/$${f}/*.elf ; done

.SECONDEXPANSION:	# required for secondary expansions of $$*
% : examples/$$*/$$*.elf
	
	
.SECONDARY:
%.elf : $$*.ino $(SRC) $(HDRS)
	$(eval OUT_PATH := $(shell dirname $@))
	$(eval BASENAME := $(shell basename $@))
	arduino-cli compile -b $(FQBN) $(CLI_ARGS) $(LIBARGS) -e $*.ino
	mv $(OUT_PATH)/build/$(subst :,.,$(FQBN))/*.elf $@
	



