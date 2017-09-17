#
# OMNeT++/OMNEST Makefile for libmyMoBAN
#
# This file was generated with the command:
#  opp_makemake -f --deep --make-so -O out -IC:/Users/zhh/Desktop/omnet++/myMiXiM/base/utils -IC:/Users/zhh/Desktop/omnet++/myMiXiM/base/modules -IC:/Users/zhh/Desktop/omnet++/myMiXiM/base/messages -IC:/Users/zhh/Desktop/omnet++/myMiXiM/base/phyLayer -IC:/Users/zhh/Desktop/omnet++/myMiXiM/base/connectionManager -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/base -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/modules -LC:/Users/zhh/Desktop/omnet++/myMiXiM/out/$(CONFIGNAME)/tests/testUtils -lmiximbase -lmiximmodules -lmiximtestUtils -KMYMIXIM_PROJ=C:/Users/zhh/Desktop/omnet++/myMiXiM
#

# Name of target to be created (-o option)
TARGET = libmyMoBAN$(SHARED_LIB_SUFFIX)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I$(MYMIXIM_PROJ)/base/utils \
    -I$(MYMIXIM_PROJ)/base/modules \
    -I$(MYMIXIM_PROJ)/base/messages \
    -I$(MYMIXIM_PROJ)/base/phyLayer \
    -I$(MYMIXIM_PROJ)/base/connectionManager \
    -I. \
    -Imobility

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L$(MYMIXIM_PROJ)/out/$(CONFIGNAME)/base -L$(MYMIXIM_PROJ)/out/$(CONFIGNAME)/modules -L$(MYMIXIM_PROJ)/out/$(CONFIGNAME)/tests/testUtils  -lmiximbase -lmiximmodules -lmiximtestUtils
LIBS += -Wl,-rpath,`abspath $(MYMIXIM_PROJ)/out/$(CONFIGNAME)/base` -Wl,-rpath,`abspath $(MYMIXIM_PROJ)/out/$(CONFIGNAME)/modules` -Wl,-rpath,`abspath $(MYMIXIM_PROJ)/out/$(CONFIGNAME)/tests/testUtils`

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = modules
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = $O/mobility/MoBANCoordinator.o $O/mobility/MoBANLocal.o $O/mobility/Posture.o $O/mobility/PostureTransition.o

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

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $(TARGET)

$(TARGET) : $O/$(TARGET)
	$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	$(SHLIB_LD) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(LIBS) $(OMNETPP_LIBS) $(LDFLAGS)
	$(SHLIB_POSTPROCESS) $O/$(TARGET)

.PHONY:

.SUFFIXES: .cc

$O/%.o: %.cc
	@$(MKPATH) $(dir $@)
	$(CXX) -c $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	-rm -rf $O
	-rm -f myMoBAN myMoBAN.exe libmyMoBAN.so libmyMoBAN.a libmyMoBAN.dll libmyMoBAN.dylib
	-rm -f ./*_m.cc ./*_m.h
	-rm -f mobility/*_m.cc mobility/*_m.h

cleanall: clean
	-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc mobility/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/mobility/MoBANCoordinator.o: mobility/MoBANCoordinator.cc \
	$(MYMIXIM_PROJ)/base/utils/Coord.h \
	mobility/MoBANCoordinator.h \
	$(MYMIXIM_PROJ)/base/utils/ImNotifiable.h \
	$(MYMIXIM_PROJ)/base/utils/HostState.h \
	$(MYMIXIM_PROJ)/base/modules/BaseUtility.h \
	$(MYMIXIM_PROJ)/base/modules/BaseWorldUtility.h \
	mobility/PostureTransition.h \
	mobility/Posture.h \
	$(MYMIXIM_PROJ)/base/utils/FWMath.h \
	$(MYMIXIM_PROJ)/base/modules/Blackboard.h \
	$(MYMIXIM_PROJ)/base/utils/FindModule.h \
	mobility/MoBANBBItem.h
$O/mobility/MoBANLocal.o: mobility/MoBANLocal.cc \
	$(MYMIXIM_PROJ)/base/utils/Coord.h \
	$(MYMIXIM_PROJ)/base/utils/ImNotifiable.h \
	$(MYMIXIM_PROJ)/base/utils/HostState.h \
	$(MYMIXIM_PROJ)/base/modules/BaseMobility.h \
	$(MYMIXIM_PROJ)/base/modules/BatteryAccess.h \
	$(MYMIXIM_PROJ)/base/modules/BaseWorldUtility.h \
	$(MYMIXIM_PROJ)/base/utils/FWMath.h \
	$(MYMIXIM_PROJ)/base/modules/Blackboard.h \
	$(MYMIXIM_PROJ)/base/modules/BaseModule.h \
	$(MYMIXIM_PROJ)/base/modules/BaseUtility.h \
	mobility/MoBANLocal.h \
	$(MYMIXIM_PROJ)/base/modules/BaseBattery.h \
	$(MYMIXIM_PROJ)/base/utils/Move.h \
	mobility/MoBANBBItem.h \
	$(MYMIXIM_PROJ)/base/utils/FindModule.h
$O/mobility/Posture.o: mobility/Posture.cc \
	$(MYMIXIM_PROJ)/base/utils/Coord.h \
	$(MYMIXIM_PROJ)/base/utils/ImNotifiable.h \
	$(MYMIXIM_PROJ)/base/modules/BaseWorldUtility.h \
	mobility/Posture.h \
	$(MYMIXIM_PROJ)/base/modules/Blackboard.h \
	$(MYMIXIM_PROJ)/base/utils/FWMath.h
$O/mobility/PostureTransition.o: mobility/PostureTransition.cc \
	$(MYMIXIM_PROJ)/base/utils/Coord.h \
	$(MYMIXIM_PROJ)/base/utils/ImNotifiable.h \
	$(MYMIXIM_PROJ)/base/modules/BaseWorldUtility.h \
	mobility/PostureTransition.h \
	$(MYMIXIM_PROJ)/base/utils/FWMath.h \
	$(MYMIXIM_PROJ)/base/modules/Blackboard.h

