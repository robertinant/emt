/*
 *  ======== Package.init ========
 *  Called at initialization time (before user config script runs)
 */
function init()
{
    if (xdc.om.$name != 'cfg') {
        return;
    }

    /* the following section is needed to work around an problem in the rts
     * packages for arm: both TI and Gnu boot files define a .bootVects
     * section, but only TI creates the section below (if it's not already
     * defined).
     * 
     * The issue is that, if the TI rts package defines this sectios it _also_ 
     * places this section at 0 but the CC3200 does not have 0 in the link
     * memory map(!) this triggers a link error.
     * 
     * In order to link with SYS/BIOS, we should set the type to "DESCT" 
     * because SYS/BIOS will replces this section with its own.
     * 
     * In order to link without SYS/BIOS, type should not be set and the
     * address should be set to 0x20004000
     */

    if (Program.sectMap[".bootVecs"] === undefined) {
        Program.sectMap[".bootVecs"] = new Program.SectionSpec();
        Program.sectMap[".bootVecs"].loadAddress = 0x20004000;
    }
}

/*
 *  ======== Package.getLibs ========
 *  Called during generation to get libraries to link with
 */
function getLibs(prog)
{
    /* get Platform module */
    var Platform = xdc.om[this.$name + ".Platform"];

    var libs;
    if (Platform.$private.src) {
	/* get the output directory */
	var outDir = Platform.$private.src.getGenSourceDir();

	/* convert to absolute path since it's usually _not_ in this package */
	outDir = String(java.io.File(outDir).getAbsolutePath());

	/* construct appropriate lib file sequence to link with */
	libs = outDir + "/Board_init.obj";
    }

    /* get required DriverLib libraries (if requested) */
    var dlib = Platform.addDriverLibs ? Platform.findDriverLib() : null;
    if (dlib != null) {
	libs = (libs ? (libs + ';') : "")
	    + dlib + "/driverlib/gcc/Release/driverlib.a";
    }

    /* '!' tells XDCtools the file won't exist until _after_ generation */
    return (libs ? ('!' + libs) : null);
}
