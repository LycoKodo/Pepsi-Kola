# Host testbench for the drive code. `make` builds one binary per DUT:
#   make DUT=live       -> build/live/tb_live           (sources: src/lemlib, as edited now)
#   make DUT=<name>     -> build/<name>/tb_<name>       (sources: tools/sim/dut_<name>/, laid out like src/lemlib)
#   e.g. DUT=baseline (snapshot at the first session's HEAD), DUT=fix1 (snapshot with the PID fix)
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
BENCH_FILES := world.cpp stubs_impl.cpp pose_mirror.cpp tb_main.cpp

# every DUT there is: live plus each tools/sim/dut_<name>/
DUTS := live $(sort $(patsubst dut_%,%,$(notdir $(wildcard $(SIM)/dut_*))))

.PHONY: all clean $(DUTS)
ifeq ($(DUT),)
all: $(DUTS)
$(DUTS):
	@$(MAKE) --no-print-directory -f $(SIM)/Makefile DUT=$@
clean:
	rm -rf $(SIM)/build
else

ifeq ($(DUT),live)
DUT_SRC := $(REPO)/src/lemlib
else
DUT_SRC := $(SIM)/dut_$(DUT)
ifeq ($(wildcard $(DUT_SRC)/chassis/motions/moveToPose.cpp),)
$(error $(DUT_SRC) does not look like src/lemlib (no chassis/motions/moveToPose.cpp))
endif
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
