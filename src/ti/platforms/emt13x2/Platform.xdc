/*
 *  ======== Platform.xdc ========
 */

/*
 *  ======== Platform ========
 */
metaonly module Platform inherits ti.platforms.launchpad.IPlatform
{
    override config xdc.platform.IPlatform.Board BOARD = {
        id:             "0",
        boardName:      "CC13x2 LaunchPad",
        boardFamily:    "CC13x2_LaunchPad",
        boardRevision:  "1.0"
    };

    /*!
     *  ======== driverLibPattern ========
     *  The pattern used to locate CC13x2 DriverLib
     *
     *  The default setting below locates driverLib relative to the TI-RTOS
     *  repo which must be on the package path in order to access reentrant
     *  SYS/BIOS peripheral drivers.
     *
     *  Note: this default may be dynamically changed in this module's
     *  module$meta$init() startup function (defined in Platform.xs).
     *
     *  @see ti.platforms.launchpad.IPlatform#driverLibPattern
     */
    override config String driverLibPattern = "ti/devices/cc13x2_cc26x2";

instance:

    /*
     *  This platform supports CC132x2 devices with Cortex M4 cores.
     */
    override config xdc.platform.IExeContext.Cpu CPU = {
        id:             "0",
        clockRate:      48.0,
        catalogName:    "ti.catalog.arm.cortexm4",
        deviceName:     "CC13xx",
        revision:       "",
    };

    override config String variant = "CC1352R1_LAUNCHXL";
}

