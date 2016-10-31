#
#  This file defines the installation location for products that are required
#  for the use of the Energia MT
#
#  XDCROOT        - XDCtools used to build a configuration of TI-RTOS
#                   suitable for EMT
#  TIRTOS         - TI-RTOS
#  DRVLIB.msp432  - MSP432 driverlib
#  DRVLIB.cc3200  - CC3200 driverlib
#  DRVLIB.cc26xx  - CC26xx driverlib
#  ti.targets.arm.elf.M4F - TI Arm compiler
#  gnu.targets.arm.M4F    - Gnu Arm compiler
#

TREE_ROOT = $(firstword $(subst /src/, /src/,$(CURDIR)))

# function to get latest versioned path
latest = $(lastword $(sort $(wildcard $1)))

#
#  CCS build support
#
CCSROOT ?= c:/ti
CCSROOT := $(subst \,/,$(CCSROOT))

TIRTOS  ?= $(call latest, c:/ti/tirtos_msp430_2_12_*)
XDCROOT  = $(call latest, $(CCSROOT)/xdctools_*/.)
INO2CPP  = $(call latest, $(CCSROOT)/energia-0101E0018*/.)/tools/ino2cpp/ino2cpp.sh

DRVLIB.msp432 = $(wildcard $(TIRTOS)/products/MSPWare*)
DRVLIB.cc3200 = $(wildcard $(TIRTOS)/products/CC32*)
DRVLIB.cc26xx = $(wildcard $(TIRTOS)/products/cc26*)
DRVLIB.cc13xx = $(wildcard $(TIRTOS)/products/cc13*)

ti.targets.arm.elf.M4F = $(call latest,$(CCSROOT)/ccsv6/tools/compiler/*arm_5.*)
gnu.targets.arm.M4F    = $(call latest,$(CCSROOT)/ccsv6/tools/compiler/gcc-arm-*)

ifeq (,$(XDCROOT))
    #
    # UNIX TISB tree build support
    #
    INO2CPP = $(TREE_ROOT)/imports/ino2cpp/ino2cpp.sh
    XDCROOT = $(TOOLS)/vendors/xdc/xdctools_3_32_01_10_eng/$(BUILD_HOST_OS)
    TIRTOS  = $(firstword $(wildcard $(TREES)/zumaprod/zumaprod-o06/exports/tirtos_full_*))
    SYSBIOS = $(wildcard $(TIRTOS)/products/bios_6_*)
    TIDRIVERS = $(wildcard $(TIRTOS)/products/tidrivers_*)

    DRVLIB.msp432 = $(wildcard $(TIRTOS)/products/msp432_driverlib*)
    DRVLIB.cc3200 = $(wildcard $(TIRTOS)/products/CC32*)
    DRVLIB.cc26xx = $(wildcard $(TIRTOS)/products/cc26*)
    DRVLIB.cc13xx = $(wildcard $(TIRTOS)/products/cc13*)
    ti.targets.arm.elf.M4F = $(TOOLS)/vendors/ti/arm/5.2.2/$(BUILD_HOST_OS)
    gnu.targets.arm.M4F    = $(TOOLS)/vendors/linaro/4.8-2014q3/$(BUILD_HOST_OS)
endif

ti.targets.arm.elf.M4  = $(ti.targets.arm.elf.M4F)
gnu.targets.arm.M4     = $(gnu.targets.arm.M4F)
ti.targets.arm.elf.M3  = $(ti.targets.arm.elf.M4F)
gnu.targets.arm.M3     = $(gnu.targets.arm.M4F)

#
#  Error checks
#
ifeq (,$(wildcard $(XDCROOT)))
    $(error XDCROOT, '$(XDCROOT)', does not reference a valid directory)
endif

ifneq (,$(filter-out clean .clean,$(MAKECMDGOALS)))
  ifeq (,$(wildcard $(INO2CPP)))
    $(warning INO2CPP, '$(INO2CPP)', does not reference a valid file)
  endif
  ifeq (,$(wildcard $(TIRTOS)))
    $(error TIRTOS, '$(TIRTOS)', does not reference a valid directory)
  endif
  ifeq (,$(wildcard $(DRVLIB.msp432)))
    $(error DRVLIB.msp432, '$(DRVLIB.msp432)', does not reference a valid directory)
  endif
  ifeq (,$(wildcard $(DRVLIB.cc3200)))
    $(error DRVLIB.cc3200, '$(DRVLIB.cc3200)', does not reference a valid directory)
  endif
  ifeq (,$(wildcard $(DRVLIB.cc26xx)))
    $(error DRVLIB.cc26xx, '$(DRVLIB.cc26xx)', does not reference a valid directory)
  endif
  ifeq (,$(wildcard $(DRVLIB.cc13xx)))
    $(error DRVLIB.cc13xx, '$(DRVLIB.cc13xx)', does not reference a valid directory)
  endif
endif
