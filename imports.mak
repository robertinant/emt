#
# XDCtools used to build this tree
#
XDCROOT = $(TOOLS)/vendors/xdc/xdctools_3_32_01_10_eng/$(BUILD_HOST_OS)

#
# internal TISB trees containing imports
#
ZUMAPROD_TREE = $(TREES)/zumaprod/zumaprod-j04

#
# Specific required imports (referenced by makeunix, for example)
#
TIRTOS  = $(firstword $(wildcard $(ZUMAPROD_TREE)/exports/tirtos_full_*))
SYSBIOS = $(wildcard $(TIRTOS)/products/bios_6*)
TIDRIVERS = $(wildcard $(TIRTOS)/products/tidrivers_full_*)
#TIDRIVERS = /db/vtree/dr/drivers-dr/exports/tidrivers_full_2_15_00_26

#
# SM-MAKE support
#
IMPORT_ARCHIVES = 
IMPORT_PACKAGES =

REFERENCED_REPOS = $(TIRTOS) $(SYSBIOS)
