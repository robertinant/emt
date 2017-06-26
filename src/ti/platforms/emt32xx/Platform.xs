/*
 *  ======== Platform.xs ========
 *  Platform support for the CC32xx Launch Pad
 */

/*
 *  ======== Platform.instance$meta$init ========
 *  This function is called to initialize a newly created instance of a
 *  platform.  Platform instances are created just prior to running
 *  program configuration scripts.
 *
 *  Platform instances may also be created in the build domain.
 *
 *  @param(name)        the name used to identify this instance (without
 *                      the package name prefix)
 *
 */
function instance$meta$init(name)
{
    this.includeLinkCmdFile = true;
    if (name != "") {
	this.variant = name;
    }

    /* We use variant name to compute the device */
    if (name.match(/CC3220SF/)) {
        this.deviceName = "CC3220SF";
    }
    else if (name.match(/CC3220S/)) {
        this.deviceName = "CC3220S";
    }
    else {
        this.deviceName = "CC3220SF";
        print(this.$package + ": Unrecognized device: '" + name + "', using "
             + this.deviceName);
    }

    /* We use 'revision' to pass 'the real device name' to the generic
     * CC32xx device.
     */
    this.CPU.revision = this.deviceName;
    this.CPU.catalogName = "ti.catalog.arm.cortexm4";
    this.CPU.clockRate = 80;

    if (Program.build.target.$name.match(/gnu/)) {
        this.codeMemory = "REGION_TEXT";
        this.dataMemory = "REGION_DATA";
        this.stackMemory = "REGION_STACK";
    }
}
