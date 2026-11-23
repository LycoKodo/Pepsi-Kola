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
BENCH_FILES := world.cpp stubs_impl.cpp pose_mirror.cpp gps_model.cpp dist_model.cpp route.cpp tb_main.cpp

# every DUT there is: live plus each tools/sim/dut_<name>/
DUTS := live $(sort $(patsubst dut_%,%,$(notdir $(wildcard $(SIM)/dut_*))))

.PHONY: all clean unit $(DUTS)
ifeq ($(DUT),)
all: $(DUTS)
$(DUTS):
	@$(MAKE) --no-print-directory -f $(SIM)/Makefile DUT=$@
clean:
	rm -rf $(SIM)/build

# `make unit`: build and run the GPS fusion tests against src/lemlib/chassis/gpsFusion.cpp.
#  1. tests/gps_fusion_unit.cpp: the filter (GpsFusionCore). GPSFUSION_CORE_ONLY leaves out the file's PROS
#     half, and only the repo's lemlib headers are on the include path (no stubs): that it builds at all shows
#     the filter needs no PROS
#  2. tests/gps_fusion_walls.cpp: the wall fixes in the same filter (docs/WALL_FIX_DESIGN.md), built the same way
#  3. tests/gps_fusion_layer.cpp: the robot's half (configure, configureWalls, setMode, status, step, the
#     [GPSFUSE] and [WALLFIX] lines), built against the stub PROS headers in stubs/
#  4. tests/dist_model_test.cpp: the bench's simulated distance sensor (bench/dist_model.cpp) against
#     hand-computed numbers; not the filter, but the wall tests in wall_suite.py stand on it
# GPSFUSION_SRC can point the unit tests at another copy of the filter (the mutation tests do)
GPSFUSION_SRC ?= $(REPO)/src/lemlib/chassis/gpsFusion.cpp
UNIT_TB  := $(SIM)/build/unit/gps_fusion_unit
WALLS_TB := $(SIM)/build/unit/gps_fusion_walls
LAYER_TB := $(SIM)/build/unit/gps_fusion_layer
DISTM_TB := $(SIM)/build/unit/dist_model_test
unit: | $(INC)/lemlib
	@mkdir -p $(dir $(UNIT_TB))
	$(CXX) $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -Wall -Wextra -DGPSFUSION_CORE_ONLY -I$(INC) \
	    $(SIM)/tests/gps_fusion_unit.cpp $(GPSFUSION_SRC) -o $(UNIT_TB)
	$(CXX) $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -Wall -Wextra -DGPSFUSION_CORE_ONLY -I$(INC) \
	    $(SIM)/tests/gps_fusion_walls.cpp $(GPSFUSION_SRC) -o $(WALLS_TB)
	$(CXX) $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -Wall -Wextra -DSIM_DIR='"$(SIM)"' -I$(SIM)/stubs \
	    -I$(INC) $(SIM)/tests/gps_fusion_layer.cpp $(GPSFUSION_SRC) -o $(LAYER_TB)
	$(CXX) $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -Wall -Wextra -I$(SIM)/bench \
	    $(SIM)/tests/dist_model_test.cpp $(SIM)/bench/dist_model.cpp -o $(DISTM_TB)
	$(UNIT_TB)
	$(WALLS_TB)
	$(LAYER_TB)
	$(DISTM_TB)

$(INC)/lemlib:
	@mkdir -p $(INC)
	ln -sfn $(REPO)/include/lemlib $(INC)/lemlib
else

ifeq ($(DUT),live)
DUT_SRC := $(REPO)/src/lemlib
else
DUT_SRC := $(SIM)/dut_$(DUT)
ifeq ($(wildcard $(DUT_SRC)/chassis/motions/moveToPose.cpp),)
$(error $(DUT_SRC) does not look like src/lemlib (no chassis/motions/moveToPose.cpp))
endif
endif

# GPS fusion (docs/GPS_FUSION_DESIGN.md): only DUTs from after it was added have it. The bench's route mode
# (bench/route.cpp) only calls the fusion functions when DUT_HAS_GPSFUSION is 1
HAS_GPSFUSION := $(if $(wildcard $(DUT_SRC)/chassis/gpsFusion.cpp),1,0)
DUT_FILES += $(if $(filter 1,$(HAS_GPSFUSION)),chassis/gpsFusion.cpp)
# The IMU heading scale (lemlib::setImuScale in chassis/odom.cpp): only DUTs from after it was added have it. The
# bench's --imu-scale calls it only when DUT_HAS_IMUSCALE is 1 (the UMBmark square, umb_suite.py)
HAS_IMUSCALE := $(if $(shell grep -l 'setImuScale' $(DUT_SRC)/chassis/odom.cpp 2>/dev/null),1,0)

BUILD := $(SIM)/build/$(DUT)
TB    := $(BUILD)/tb_$(DUT)
COMMON := $(SDKFLAG) -std=c++20 -O2 -g -ffp-contract=off -MMD -MP -I$(SIM)/stubs -I$(INC)
# the DUT is not ours to clean up: no warnings. PROS adds -iquote include/<dir of the file>, which
# is how pid.cpp's #include "pid.hpp" resolves; do the same
DUT_FLAGS   := $(COMMON) -w
BENCH_FLAGS := $(COMMON) -Wall -Wextra -Wno-unused-parameter -iquote $(SIM)/bench \
               -DSIM_DIR='"$(SIM)"' -DDUT_NAME='"$(DUT)"' -DDUT_HAS_GPSFUSION=$(HAS_GPSFUSION) \
               -DDUT_HAS_IMUSCALE=$(HAS_IMUSCALE)

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
