#
# OMNeT++/OMNEST Makefile for myMoBAN
#
# This file was generated with the command:
#  opp_makemake -f --deep --nolink -O out -d examples -d modules -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/base -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/modules -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/tests/testUtils -L./out/$(CONFIGNAME)/modules -lmiximbase -lmiximmodules -lmiximtestUtils -KMYMIXIM_PROJ=C:/Users/zhh/Desktop/omnet++/myMiXiM
#

# C++ include paths (with -I)
INCLUDE_PATH = -I. -Idoc -Idoc/Figures -Idoc/doxy -Idoc/neddoc

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS =

# Message files
MSGFILES =

# Other makefile variables (-K)
MYMIXIM_PROJ=C:/Users/zhh/Desktop/omnet++/myMiXiM

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target

all: $(OBJS) submakedirs Makefile
	@# Do nothing

submakedirs:  examples_dir modules_dir

.PHONY: examples modules
examples: examples_dir
modules: modules_dir

examples_dir:
	cd examples && $(MAKE)

modules_dir:
	cd modules && $(MAKE)

.SUFFIXES: .cc

$O/%.o: %.cc
	@$(MKPATH) $(dir $@)
	$(CXX) -c $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)
	cd examples && $(MAKE) msgheaders
	cd modules && $(MAKE) msgheaders

clean:
	-rm -rf $O
	-rm -f myMoBAN myMoBAN.exe libmyMoBAN.so libmyMoBAN.a libmyMoBAN.dll libmyMoBAN.dylib
	-rm -f ./*_m.cc ./*_m.h
	-rm -f doc/*_m.cc doc/*_m.h
	-rm -f doc/Figures/*_m.cc doc/Figures/*_m.h
	-rm -f doc/doxy/*_m.cc doc/doxy/*_m.h
	-rm -f doc/neddoc/*_m.cc doc/neddoc/*_m.h

	-cd examples && $(MAKE) clean
	-cd modules && $(MAKE) clean

cleanall: clean
	-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc doc/*.cc doc/Figures/*.cc doc/doxy/*.cc doc/neddoc/*.cc
	-cd examples && if [ -f Makefile ]; then $(MAKE) depend; fi
	-cd modules && if [ -f Makefile ]; then $(MAKE) depend; fi

# DO NOT DELETE THIS LINE -- make depend depends on it.

