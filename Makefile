################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
# libcoldimages.a is excluded here only so the wildcard can't put it in a different position from one build
# to the next (that would move every cold address and force a cold re-upload); it is added to the cold link
# explicitly at the bottom of this file
EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/libcoldimages.a $(FWDIR)/liblemlib.a

# LemLib is built from src/lemlib (see the bottom of this file) and linked into the cold package, so its
# sources are kept out of the normal (hot) program here. selector.cpp and smartMotor.cpp are team code
# that happens to live in that folder: they use the robot's own files, so they stay in the hot program
LEMLIB_HOT_SRC:=$(SRCDIR)/lemlib/selector.cpp $(SRCDIR)/lemlib/smartMotor.cpp
EXCLUDE_SRCDIRS+=$(filter-out $(LEMLIB_HOT_SRC),$(wildcard $(SRCDIR)/lemlib/*))

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

################################################################################
# Screen images go in the COLD package (team addition)
#
# The two LVGL images in cold/ are about 1.35 MB, nearly all of the program. They
# almost never change, so they are built into a library that links into the cold
# package (uploaded once, kept on the brain) instead of the hot package (uploaded
# every time). That keeps each normal upload small enough to send through the
# controller's radio. After changing an image, the next upload re-sends the cold
# package once (big: do that one over the cable).
#
# Code in src/ uses them as before (LV_IMG_DECLARE(image) / LV_IMG_DECLARE(dorito)).
################################################################################
COLD_IMG_SRC:=$(wildcard $(ROOT)/cold/*.c)
COLD_IMG_OBJ:=$(patsubst $(ROOT)/cold/%.c,$(BINDIR)/cold/%.c.o,$(COLD_IMG_SRC))
COLD_IMG_LIB:=$(FWDIR)/libcoldimages.a

$(BINDIR)/cold/%.c.o: $(ROOT)/cold/%.c
	$(VV)mkdir -p $(dir $@)
	$(call test_output_2,Compiled $< ,$(CC) -c $(INCLUDE) $(CFLAGS) $(EXTRA_CFLAGS) -o $@ $<,$(OK_STRING))

$(COLD_IMG_LIB): $(COLD_IMG_OBJ)
	$(VV)rm -f $@
	$(call test_output_2,Archiving cold images ,$(AR) rcs $@ $^,$(DONE_STRING))

################################################################################
# LemLib from source, in the COLD package (team addition)
#
# The prebuilt firmware/LemLib.a used to be linked next to src/lemlib, and for every function in both
# the prebuilt one won, so edits in src/lemlib never ran. It is no longer linked (kept, unused, as
# firmware/prebuilt/LemLib.a.unused). src/lemlib is now the library that runs: it is compiled into
# firmware/liblemlib.a and linked into the cold package, so normal uploads stay small. After changing
# anything under src/lemlib (or include/lemlib), the next upload re-sends the cold package once (big:
# do that one over the cable).
################################################################################
LEMLIB_SRC:=$(filter-out $(LEMLIB_HOT_SRC),$(call rwildcard,$(SRCDIR)/lemlib/,*.cpp))
LEMLIB_OBJ:=$(patsubst $(SRCDIR)/%,$(BINDIR)/%.o,$(LEMLIB_SRC))
LEMLIB_LIB:=$(FWDIR)/liblemlib.a

$(LEMLIB_LIB): $(LEMLIB_OBJ)
	$(VV)rm -f $@
	$(call test_output_2,Archiving LemLib from source ,$(AR) rcs $@ $^,$(DONE_STRING))

# The cold link uses --whole-archive on all its prerequisites, so this pulls the images and LemLib in
$(COLD_ELF): $(COLD_IMG_LIB) $(LEMLIB_LIB)
