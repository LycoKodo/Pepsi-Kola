# Host testbench for the drive code. `make` builds both binaries:
#   make DUT=baseline   -> build/baseline/tb_baseline   (sources: tools/sim/dut_baseline, the snapshot)
#   make DUT=live       -> build/live/tb_live           (sources: src/lemlib, as edited now)
# Headers always come from the repo's include/lemlib (through build/inc), after the stubs in stubs/.
# The DUT files are compiled unmodified; header changes trigger rebuilds (-MMD).

SIM  := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
REPO := $(abspath $(SIM)/../..)
CXX  := /usr/bin/clang++
INC  := $(SIM)/build/inc

# This laptop's default SDK (MacOSX27.0) is newer than its linker, which then rejects libSystem.tbd.
# Use the default SDK if a test program links, otherwise the newest installed SDK that does.
# Override with `make SDKFLAG=-isysroot/path/to/MacOSX.sdk` (or SDKFLAG= for the default).
SDKFLAG ?= $(shell d=$$(mktemp -d); echo 'int main(){return 0;}' > $$d/t.cpp; \
  if ! $(CXX) $$d/t.cpp -o $$d/t >/dev/null 2>&1; then \
    for s in $$(ls -d /Library/Developer/CommandLineTools/SDKs/MacOSX[0-9]*.sdk 2>/dev/null | sort -r); do \
      if $(CXX) -isysroot $$s $$d/t.cpp -o $$d/t >/dev/null 2>&1; then echo "-isysroot $$s"; break; fi; \
    done; \
  fi; rm -rf $$d)
export SDKFLAG

DUT_FILES := chassis/motions/moveToPoint.cpp chassis/motions/turnToHeading.cpp chassis/motions/moveToPose.cpp \
             chassis/chassis.cpp chassis/odom.cpp chassis/trackingWheel.cpp pid.cpp exitcondition.cpp timer.cpp \
             util.cpp pose.cpp driveCurve.cpp
BENCH_FILES := world.cpp stubs_impl.cpp tb_main.cpp

.PHONY: all baseline live clean
ifeq ($(DUT),)
all: baseline live
baseline:
	@$(MAKE) --no-print-directory -f $(SIM)/Makefile DUT=baseline
live:
	@$(MAKE) --no-print-directory -f $(SIM)/Makefile DUT=live
clean:
	rm -rf $(SIM)/build
else

ifeq ($(DUT),baseline)
DUT_SRC := $(SIM)/dut_baseline
else ifeq ($(DUT),live)
DUT_SRC := $(REPO)/src/lemlib
else
$(error DUT must be baseline or live)
endif

BUILD := $(SIM)/build/$(DUT)
TB    := $(BUILD)/tb_$(DUT)
COMMON := $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -MMD -MP -I$(SIM)/stubs -I$(INC)
# the DUT is not ours to clean up: no warnings. PROS adds -iquote include/<dir of the file>, which
# is how pid.cpp's #include "pid.hpp" resolves; do the same
DUT_FLAGS   := $(COMMON) -w
BENCH_FLAGS := $(COMMON) -Wall -Wextra -Wno-unused-parameter -iquote $(SIM)/bench \
               -DSIM_DIR='"$(SIM)"' -DDUT_NAME='"$(DUT)"'

DUT_OBJS   := $(addprefix $(BUILD)/dut/,$(DUT_FILES:.cpp=.o))
BENCH_OBJS := $(addprefix $(BUILD)/bench/,$(BENCH_FILES:.cpp=.o))

all: $(TB)

$(TB): $(DUT_OBJS) $(BENCH_OBJS)
	$(CXX) $(SDKFLAG) -o $@ $^
	@echo "built $@ from $(DUT_SRC)"

$(INC)/lemlib:
	@mkdir -p $(INC)
	ln -sfn $(REPO)/include/lemlib $(INC)/lemlib

$(BUILD)/dut/%.o: $(DUT_SRC)/%.cpp | $(INC)/lemlib
	@mkdir -p $(dir $@)
	$(CXX) $(DUT_FLAGS) -iquote $(INC)/lemlib/$(patsubst %/,%,$(dir $*)) -c $< -o $@

$(BUILD)/bench/%.o: $(SIM)/bench/%.cpp | $(INC)/lemlib
	@mkdir -p $(dir $@)
	$(CXX) $(BENCH_FLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD)

-include $(DUT_OBJS:.o=.d) $(BENCH_OBJS:.o=.d)
endif
