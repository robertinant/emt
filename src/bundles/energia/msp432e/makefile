#
#  Example makefile to build an MSP432E LP TI-RTOS "reusable configuration";
#  a fixed configuration of TI-RTOS that can be consumed by applications as
#  a 100% self contained set of libraries and headers.
#
#  This configuration requires the installation directory of each of the
#  following prerequisite components:
#      CCROOT  - the GCC arm compiler (version 4.8 or above)
#      XDCROOT - XDCtools (version 3.30.05 or above)
#      SDK.msp432e  - core SDK for MSP432E
#      EMTROOT - the EMT packages repo
#

# include definitions of the macros described above
include tools.mak

# define the XDC package path based on the settings above
SYSBIOS   ?= $(SDK.msp432e)/kernel/tirtos/packages
TIDRIVERS ?= $(SDK.msp432e)/source

XDCPATH = $(SYSBIOS);$(TIDRIVERS);$(EMTROOT)

PLATFORM = ti.platforms.emt432e
TARGET   = gnu.targets.arm.M4F
CCOPTS   = --compileOptions "-gdwarf-3 -gstrict-dwarf -fno-exceptions"

VARIANTS = $(EMTROOT)/ti/runtime/wiring/msp432e/variants

ifneq (,$(wildcard $(XDCROOT)/bin/rm.exe))
    RMDIR = $(XDCROOT)/bin/rm -rf
else
    RMDIR = rm -rf
endif

all: .debug closure

# output all prerequisites used to create the configuration
.debug:
	@echo "  ---- imports ----"
	@echo "    CCROOT  = $(CCROOT)"
	@echo "    XDCROOT = $(XDCROOT)"
	@echo "    SDK     = $(SDK.msp432e)"
	@echo "    SYSBIOS = $(SYSBIOS)"
	@echo "    EMTROOT = $(EMTROOT)"
	@echo "    XDCPATH = $(XDCPATH)"
	@echo "  -----------------"
	@echo ""

# create a single directory containing _everything_ referenced by configPkg
closure: configPkg
	@$(RMDIR) $@
	@echo xs xdc.tools.closure -d $@ $< ...
	$(XDCROOT)/xs xdc.tools.closure -d $@ -i ti.drivers.bsp $<

	$(XDCROOT)/bin/xdccpp $(wildcard $(VARIANTS)/*) $@

	@echo patching $@ ...
	@$(CURDIR)/../patch.ksh $@ ./src $<

# create a fixed configuration of TI-RTOS from energia.cfg
configPkg: energia.cfg
	@echo xs xdc.tools.configuro -o $@ $< ...
	$(XDCROOT)/xs --xdcpath="$(XDCPATH)" xdc.tools.configuro -o $@ -t $(TARGET) $(CCOPTS) -p $(PLATFORM) -r release -c $(CCROOT) $<

# remove all generated files/directories
clean:
	@$(RMDIR) closure configPkg src closure.zip
