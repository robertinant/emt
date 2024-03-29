/*
 *  ======== Package.init ========
 *  Called at initialization time (before user config script runs)
 */
function init()
{
    if (xdc.om.$name != 'cfg') {
        return;
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

    if (true) {
	/* get required DriverLib libraries (if requested) */
	var dlib = Platform.addDriverLibs ? Platform.findDriverLib() : null;
	if (dlib != null) {
	    libs = (libs ? (libs + ';') : "")
		+ dlib + "/driverlib/bin/gcc/driverlib.lib";
	}
    }
    else {
	print("    WARNING: force use of pre-built DriverLib libraries");
	/* force use of pre-built DriverLib libraries (because there is 
	 * no GCC supported release) 
	 */
	var dlib = xdc.findFile("ti/runtime/driverlib/cc26xx");
	if (dlib == null) {
	    throw new Error("can't find 'ti/runtime/driverlib/cc26xx'");
	}
	libs = (libs ? (libs + ';') : "") + (dlib + "/lib/cc26xx.m3g.lib");
    }

    /* '!' tells XDCtools the file won't exist until _after_ generation */
    return (libs ? ('!' + libs) : null);
}
