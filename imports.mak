#
# XDCtools used to build this tree
#
XDCROOT = $(TOOLS)/vendors/xdc/xdctools_3_50_02_20_core/$(BUILD_HOST_OS)

ENERGIA = /db/builds/xinstall/energia_nightly/Energia

#
# internal TISB trees containing imports
#
MCPI_TREE = $(TREES)/mcpi/mcpi-3.30.00.13
SDK_CC32XX = simplelink_cc32xx_sdk_1_50_00_00_eng
SDK_MSP432 = simplelink_msp432p4_sdk_1_50_00_06

#
# Official SDK builds:
#   MSP432: http://msp430.sc.ti.com/component_builds/msp432_sdk/ 
#   CC3220: http://msp430.sc.ti.com/component_builds/CC32XX_SDK/Version
#

#
# Specific required imports (referenced by makeunix, for example)
#
SDK.msp432 = $(TOOLS)/vendors/ti/msp432_sdk/$(SDK_MSP432)
SDK.cc13xx = $(firstword $(wildcard $(MCPI_TREE)/exports/coresdk_cc13xx_*))
SDK.cc26xx = $(SDK.cc13xx)
SDK.cc32xx = $(TOOLS)/vendors/ti/cc3220_sdk/$(SDK_CC32XX)

TIRTOS.msp432   = $(SDK.msp432)/kernel/tirtos/packages
TIDRIVERS.msp432 = $(SDK.msp432)/source
TIRTOS.cc13xx   = $(SDK.cc13xx)/kernel/tirtos/packages
TIDRIVERS.cc13xx = $(SDK.cc13xx)/source
TIRTOS.cc26xx   = $(SDK.cc26xx)/kernel/tirtos/packages
TIDRIVERS.cc26xx = $(SDK.cc26xx)/source
TIRTOS.cc32xx   = $(SDK.cc32xx)/kernel/tirtos/packages
TIDRIVERS.cc32xx = $(SDK.cc32xx)/source

INO2CPP = $(wildcard $(ENERGIA)/app/src/processing/app/ino2cpp/exports/ino2cp*)

ifeq (,$(INO2CPP))
    $(error INO2CPP does not exist)
endif

#
# SM-MAKE support
#
IMPORT_ARCHIVES = $(INO2CPP)
IMPORT_PACKAGES =

REFERENCED_REPOS = $(TIRTOS.msp432) $(TIDRIVERS.msp432) \
                   $(TIRTOS.cc13xx) $(TIDRIVERS.cc13xx) \
                   $(TIRTOS.cc32xx) $(TIDRIVERS.cc32xx)
