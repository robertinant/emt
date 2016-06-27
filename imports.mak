#
# XDCtools used to build this tree
#
XDCROOT = $(TOOLS)/vendors/xdc/xdctools_3_32_01_11_eng/$(BUILD_HOST_OS)

ENERGIA = /db/builds/xinstall/energia_nightly/Energia

#
# internal TISB trees containing imports
#
ZUMAPROD_TREE = $(TREES)/zumaprod/zumaprod-n06

#
# Specific required imports (referenced by makeunix, for example)
#
TIRTOS  = $(firstword $(wildcard $(ZUMAPROD_TREE)/exports/tirtos_full_*))
SYSBIOS = $(wildcard $(TIRTOS)/products/bios_6*)
TIDRIVERS = $(wildcard $(TIRTOS)/products/tidrivers_full_*)

INO2CPP = $(wildcard $(ENERGIA)/app/src/processing/app/ino2cpp/exports/ino2cp*)

ifeq (,$(INO2CPP))
    $(error INO2CPP does not exist)
endif

#
# SM-MAKE support
#
IMPORT_ARCHIVES = $(INO2CPP)
IMPORT_PACKAGES =

REFERENCED_REPOS = $(TIRTOS) $(SYSBIOS)
