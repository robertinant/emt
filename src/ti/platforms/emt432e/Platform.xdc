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
        boardName:      "MSP_EXP432E401Y",
        boardFamily:    "MSP432E4_Launch_Pad",
        boardRevision:  "1.0"
    };

    /*!
     *  ======== driverLibPattern ========
     *  The pattern used to locate MSP43x DriverLib
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
    override config String driverLibPattern = "ti/devices/msp432e4";

instance:

    /*
     *  This platform supports MSP432E devices with Cortex M4F cores.
     */
    override config xdc.platform.IExeContext.Cpu CPU = {
        id:             "0",
        clockRate:      120.0,
        catalogName:    "ti.catalog.arm.cortexm4",
        deviceName:     "MSP432E",
        revision:       "",
    };

    override config String variant = "MSP_EXP432E401Y";
}

