INO2CPP = $(TREE_ROOT)/imports/ino2cpp/ino2cpp.sh

DRVLIB.msp432 = $(SDK.msp432)/source/ti/devices/msp432p4xx
DRVLIB.msp432e = $(SDK.msp432e)/source/ti/devices/msp432e4
DRVLIB.cc13xx = $(SDK.cc13xx)/source/ti/devices/cc13x0
DRVLIB.cc13x2 = $(SDK.cc13x2)/source/ti/devices/cc13x2_cc26x2/
DRVLIB.cc26xx = $(SDK.cc26xx)/source/ti/devices/cc26x0
DRVLIB.cc32xx = $(SDK.cc32xx)/source/ti/devices/cc32xx

ti.targets.arm.elf.M4F = $(TOOLS)/vendors/ti/arm/16.9.0/$(BUILD_HOST_OS)
gnu.targets.arm.M4F    = /db/vendors/linaro/gcc-arm-none-eabi-6-2017-q1-update/

ti.targets.arm.elf.M4  = $(ti.targets.arm.elf.M4F)
gnu.targets.arm.M4     = $(gnu.targets.arm.M4F)
ti.targets.arm.elf.M3  = $(ti.targets.arm.elf.M4F)
gnu.targets.arm.M3     = $(gnu.targets.arm.M4F)

