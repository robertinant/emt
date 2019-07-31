#
# XDCtools used to build this tree
#
XDCROOT = $(TOOLS)/vendors/xdc/xdctools_3_50_02_20_core/$(BUILD_HOST_OS)

ENERGIA = /db/builds/xinstall/energia_nightly/Energia

#
# internal TISB trees containing imports
#
MCPI_CC32XX_TREE  = $(TREES)/mcpi/mcpi-4.20.01.01
MCPI_MSP432_TREE  = $(TREES)/mcpi/mcpi-4.20.01.01
MCPI_MSP432E_TREE = $(TREES)/mcpi/mcpi-4.20.01.01
MCPI_CC13XX_TREE  = $(TREES)/mcpi/mcpi-4.20.01.01
MCPI_CC13X2_TREE  = $(TREES)/mcpi/mcpi-4.20.01.01

SDK_CC32XX = simplelink_cc32xx_sdk_2_40_02_00
SDK_MSP432 = simplelink_msp432p4_sdk_1_50_00_06
SDK_CC13XX = simplelink_cc13x0_sdk_1_50_00_08

#
# Official SDK builds:
#   MSP432: http://msp430.sc.ti.com/component_builds/msp432_sdk/ 
#   CC3220: http://msp430.sc.ti.com/component_builds/CC32XX_SDK/Version
#

#
# Specific required imports (referenced by makeunix, for example)
#
#SDK.msp432 = $(TOOLS)/vendors/ti/msp432_sdk/$(SDK_MSP432)
SDK.msp432 = $(firstword $(wildcard $(MCPI_MSP432_TREE)/exports/coresdk_msp432_*))
SDK.msp432e = $(firstword $(wildcard $(MCPI_MSP432E_TREE)/exports/coresdk_msp432e4_*))
#SDK.cc13xx = $(TOOLS)/vendors/ti/cc13xx_sdk/$(SDK_CC13XX)
SDK.cc13xx = $(firstword $(wildcard $(MCPI_CC13XX_TREE)/exports/coresdk_cc13xx_*))
SDK.cc13x2 = $(firstword $(wildcard $(MCPI_CC13X2_TREE)/exports/coresdk_cc13xx_*))
SDK.cc26xx = $(firstword $(wildcard $(MCPI_CC13XX_TREE)/exports/coresdk_cc13xx_*))
SDK.cc32xx = $(firstword $(wildcard $(MCPI_CC32XX_TREE)/exports/coresdk_cc32xx_*))
#SDK.cc32xx = $(TOOLS)/vendors/ti/cc3220_sdk/$(SDK_CC32XX)

TIRTOS.msp432   = $(SDK.msp432)/kernel/tirtos/packages
TIDRIVERS.msp432 = $(SDK.msp432)/source
TIRTOS.msp432e   = $(SDK.msp432e)/kernel/tirtos/packages
TIDRIVERS.msp432e = $(SDK.msp432e)/source
TIRTOS.cc13xx   = $(SDK.cc13xx)/kernel/tirtos/packages
TIDRIVERS.cc13xx = $(SDK.cc13xx)/source
TIRTOS.cc13x2   = $(SDK.cc13x2)/kernel/tirtos/packages
TIDRIVERS.cc13x2 = $(SDK.cc13x2)/source
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
                   $(TIRTOS.msp432e) $(TIDRIVERS.msp432e) \
                   $(TIRTOS.cc13xx) $(TIDRIVERS.cc13xx) \
                   $(TIRTOS.cc13x2) $(TIDRIVERS.cc13x2) \
                   $(TIRTOS.cc32xx) $(TIDRIVERS.cc32xx)
