/*
 *  ======== mapsum.xs ========
 *  Account for an executable's footprint in terms of the linked object files
 * 
 *  Usage: xs -c mapsum.xs [-e <executable>] [-t <toolchain>] [-v] \ 
 *                         [-f <object>] mapfile ...";
 * 
 *  Options:
 *     -e <exe>    - use <exe> to determine target/platform
 *
 *     -t <tools>  - specify the type of linker map file ("gnu" or "ti").  If
 *                   not specified the toolchain will be infered by the map 
 *                   file content.
 *
 *     -v          - verbose output.  More -v options increase the amount of 
 *                   information dumped
 *
 *     -f <obj>    - follow and display the "dependency chain" that causes
 *                   <obj> to be included in the executable
 *
 *     mapfile     - the map file(s) to analyze.  If multiple map files are
 *                   provided display accounting for each _and_ display the 
 *                   difference between the first and second, first and 
 *                   third, etc.
 *
 *  Linker map files provide the mapping of output sections to the set of
 *  object file "input sections" that it contains and a mapping for object 
 *  files to the thier containing library (if any).
 * 
 *  This tool uses these relations to create a mapping from libraries (or
 *  top-level object files) to the individual object file output sections that
 *  contribute to the executable's total footprint.  This relation provides a
 *  view of the footprint contribution of each library (or top-level object
 *  file).
 */
var usage = "usage: xs -c mapsum.xs [-e <executable>] [-t <toolchain>] [-v] [-f <object>] mapfile ...";

var symbolNamesGnu = {
    __UNUSED_SRAM_start__: "SRAM_UNUSED_start",
    __UNUSED_SRAM_end__: "SRAM_UNUSED_end",
    __SRAM_LENGTH__: "SRAM_length",

    __UNUSED_FLASH_start__: "FLASH_UNUSED_start",
    __UNUSED_FLASH_end__: "FLASH_UNUSED_end",
    __FLASH_LENGTH__: "FLASH_length"
};

var symbolTable = {};
var objectTable = {};
var toolChain = null;
var executable = null;
var exeAttrs = {};

var verbose = 0;

function main(arguments)
{
    var follow = {};

    for (;;) {
        var arg = arguments[0];
        if (arg == null || arg[0] != '-') {
            break;
        }

        switch (arg[1]) {
            case 'v': {
                verbose++;
                break;
            }
            case 't': {
                if (arguments[1] == null) {
                    print(usage);
                    return;
                }
                toolChain = arguments[1];
                arguments.shift();
                break;
            }
            case 'e': {
                if (arguments[1] == null) {
                    print(usage);
                    return;
                }
                executable = arguments[1];
                arguments.shift();
                break;
            }
            case 'f': {
                if (arguments[1] == null) {
                    print(usage);
                    return;
                }
                follow[arguments[1]] = 1;
                arguments.shift();
                break;
            }
            default: {
                print(usage);
                return;
            }
        }
        arguments.shift();
    }
    if (arguments[0] == null) {
        print(usage);
        return;
    }

    var sums = {};
    for (var i = 0; i < arguments.length; i++) {
        symbolTable = {};
        objectTable = {};
        var mfile = arguments[i];
        var carray = parse(mfile, executable);
        sums[mfile] = carray;

        print((i > 0 ? "\n" : "") + mfile + " summary:");
        display(carray, mfile, verbose);

        for (oname in follow) {
            printDepChains(objectTable, oname);
        }
    }

    displayDiffs(arguments[0], sums);

    //printDepChains(objectTable, "lib_a-gdtoa-gethex.o");
    //printDepChains(objectTable, "lib_a-er_gamma.o");
    //printDepChains(objectTable, "unwind-arm.o");

    //findRef(carray, "_aeabi_(ddiv|dmul|dadd|dsub|dcmp)");
}

/*
 *  ======== getTargetSuffix ========
 */
function getTargetSuffix(carray)
{
    for (var i = 0; i < carray.length; i++) {
        var name = carray[i].name.split(/\s/)[0];
        var k = name.lastIndexOf('.') + 1;
        if (k != 0) {
            var suffix = name.substring(k);
            if (suffix[0] == 'o' || suffix[0] == 'a') {
                if (suffix != "obj" && suffix != "a") {
                    return (suffix.substring(1));
                }
            }
        }
    }
    return (null);
}

/*
 *  ======== getToolChain ========
 */
function getToolChain(fileName)
{
    var result = null;
    
    var file = new java.io.BufferedReader(new java.io.FileReader(fileName));
    var line;

    for (var i = 0; i < 10 && (line = file.readLine()) != null; i++) {
        line = String(line);
        if (line.match(/^Archive member included/)) {
            result = "gnu";
            break;
        }
        else if (line.match(/^>> Linked /)) {
            result = "ti";
            break;
        }
    }
    file.close();
    
    return (result);
}

/*
 *  ======== getExeAttrs ========
 */
function getExeAttrs(exeFileName)
{
    var attrs = {};

    var what = String(Packages.xdc.services.global.What.getWhatString(exeFileName));
    var lines = what.split('\n');
    for (var i = 0; i < lines.length; i++) {
        var tokens = lines[i].split(/[ =]+/);
        if (tokens != null && tokens.length == 2) {
            attrs[tokens[0].trim()] = tokens[1];
        }
    }

    return (attrs);
}

/*
 *  ======== parse ========
 *  Return sorted array of "containers": objects with a total size
 *  and a hash of contributing sections and their size.
 * 
 *  Each element of the array is of the form:
 *   {
 *     name : <name of the "container" (library, object, ...)>, 
 *     total: <total size of the container>,
 *     sections: {<name of a contained "section">:
 *                 <size of this contained section>}
 *   } 
 */
function parse(fileName, executable)
{
    var result = null;

    /* if toolchain isn't specified, figure it out from the executable */
    if (toolChain == null && executable != null) {
        exeAttrs = getExeAttrs(executable);
        var target = exeAttrs.__TARG__;
        if (target != null) {
            toolChain = target.split('.')[0];
        }
    }

    /* if toolchain still isn't specified, figure it out from the map file */
    if (toolChain == null) {
        toolChain = getToolChain(fileName);
    }

    switch (toolChain) {
        case "gnu": {
            result = parseGnu(fileName);
            break;
        }
        case "ti": {
            result = parseTI(fileName);
            break;
        }
        default: {
            throw new Error("unknown toolchain '" 
                            + toolChain + "'; use 'ti' or 'gnu'.");
            break;
        }
    }
    return (result);
}

/*
 *  ======== parseGnu ========
 *  Heuristics:
 *     *fill* sections exist to 
 *         o align the beginning of a subsequent input section
 *         o statically allocate space for stacks, heaps, ROM data blocks
 */
function parseGnu(fileName) 
{
    var file = new java.io.BufferedReader(new java.io.FileReader(fileName));
    var line;
    var result = {};

    var header = true;
    var deps = true;
    var continuation = false;
    var fill = 0;
    while ((line = file.readLine()) != null) {
        line = String(line);

        if (line.indexOf("Linker script and memory map") == 0) {
            header = false;
        }

        /* look for symbol values for the symbols defined in symbolNamesGnu */
        if (symbolValueGnu(line, symbolNamesGnu, symbolTable)) {
            continue;
        }

        if (header) {
            if (deps) {
                deps = objectDepsGnu(line, objectTable);
            }
            continue;
        }

        /* now look for section allocation lines */
        var outSection;  /* most recent top-level output section name */
        var section;     /* output section name */
        var prevSection; /* most recent non-null section name */
        var size;        /* size of contained "section" */
        var key;         /* input container name + optional object name */
        var start;       /* start address */

        /* Look for the start of a new output section; i.e., lines of the
         *  form: 
         *     "^<output_section_name>( 0x<addr> 0x<size>)?" 
         */
        var tokens = line.match(/^([\.a-zA-Z0-9_\*]+)(\s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+))?/);
        if (tokens != null) {
            outSection = tokens[1];

            /* add outstanding *fill* to most recent elem of prev section */
            if (fill != 0) {
                var elem = result[key];
                if (elem != null) {
                    elem.fill += fill;
                    elem.total += fill;

                    /* add a separate fill section to see who needs alignment*/
                    elem.sections[prevSection + "(*fill*)"] = fill;
                    fill = 0;
                }
                else {
                    print("======= key: " + key 
                          + "\n      line: " + line
                          + "\n      prev: " + prevSection);
                }
            }
            //key = outSection;
            continue;
        }

        /* look for lines of the form:
         *       section_object start    size     key
         *    "^ <section_name> 0x<addr> 0x<size> <container>(<object>)", or
         *    "^ <section_name>",                                         or
         *    "^     0x<addr> 0x<size> <container>(<object>)", immediately
         *    followed by the "lone section name" form above
         */
        tokens = line.match(/^ ([\.a-zA-Z0-9_+:\*]+)\s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+([\.a-zA-Z0-9_+:\/\\\-]+)(\(.+\))?/);
        if (tokens == null) {

            /* check for *fill* sections, these are either
             *     o object alignment padding, or
             *     o raw space allocation: heap, stack, ...
             */
            tokens = line.match(/^\s+\*fill\*\s+0x[0-9a-f]+\s+(0x[\0-9a-f]+)/);
            if (tokens != null && tokens[1] != null && outSection.indexOf(".debug") != 0) {
                fill += tokens[1] - 0;
                continue;
            }

            if (continuation == true) {
                /* parse continuation line */
                continuation = false;
                tokens = line.match(/^ \s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+([\.a-zA-Z0-9_+:\/\\\-]+)(\(.+\))?/);
                if (tokens == null) {
                    print("warning: expected a continuation of section '" 
                          + section + "', skipping line: " + line);
                    continue;
                }
                section += (tokens[4] ? tokens[4] : "");
                start = tokens[1] - 0;
                size = tokens[2] - 0;
                key = tokens[3];
            }
            else {
                /* check for <section_name> start */
                tokens = line.match(/^ ([\.a-zA-Z0-9_+:\*]+)$/);
                if (tokens != null && tokens[1] != "CREATE_OBJECT_SYMBOLS") {
                    continuation = true;
                    section = tokens[1];
                }
                else {
                    /* don't know what this line is (do we care?) */
                    if (section != null) {
                        prevSection = section;
                        section = null;
                    }
                }
                continue;
            }
        }
        else {
            section = tokens[1] + (tokens[5] ? tokens[5] : "");
            start = tokens[2] - 0;
            size = tokens[3] - 0;
            key = tokens[4];
            continuation = false;
        }

        /* ignore zero length sections or well-known non-loadable sections */
        if (size == 0 || section.match(/^(\.(debug|comment|stab|ARM\.attr|TI\.section\.flag|TI\.symbol))|xdc\.|__TI_DW\.debug|__TI_ICODE/) != null) {
            continue;
        }

        /* treat COMMON sections as a single container */
        if (section == "COMMON") {
            section = key;
            key = "COMMON";
        }
            
        /* if we get here, we should have a "key": a container name that we
         * canonicalize
         */
        var index = Math.max(key.lastIndexOf('/'), key.lastIndexOf('\\'));
        var prefix = key.substring(0, index);
        if (java.io.File(prefix).isAbsolute()) {
            prefix = String(java.io.File(prefix).getCanonicalPath());
        }
        key = key.substring(index + 1) 
            + " (" + prefix.replace(/\\/g, '/') + ")";

        /* accumulate sizes for each "container" */
        if (result[key] == null) {
            result[key] = {name: key, total: 0, sections: {}, fill: 0};
        }
        result[key].total += size + fill;
        result[key].fill += fill;

        /* accumulate sizes of each contained section */
        var ssize = result[key].sections[section];
        result[key].sections[section] = size + (ssize != null ? ssize : 0);
        if (fill) {
            /* add a separate fill section so we can see who needs alignment */
            result[key].sections[section + "(*fill*)"] = fill;
            fill = 0;
        }
    }
    if (fill != 0) {
        print("warning: unattributed fill value: " 
              + fill + ", section: " + section);
        result["*fill*"] = {
            name: "*fill*", total: fill, sections: {}, fill: fill
        };
        fill = 0;
    }

    /* close file stream */
    file.close();

    /* create array of containers and sort by size */
    var carray = [];
    for (var k in result) {
        carray.push(result[k]);
    }
    carray.sort(function (a, b) {return (b.total - a.total);});

    return (carray);
}

/*
 *  ======== parseTI ========
 */
function parseTI(fileName) 
{
    var file = new java.io.BufferedReader(new java.io.FileReader(fileName));
    var line;
    var result = {};
    while ((line = file.readLine()) != null) {
        line = String(line);

        /* look for lines of the form "  <hex_addr> <hex_size> <anything>" */
        var tokens = line.match(/        ([0-9a-f]+)    ([0-9a-f]+)     (.*)/);
        if (tokens != null) {
            var start = ("0x" + tokens[1]) - 0;
            var size = ("0x" + tokens[2]) - 0;
            var rest = tokens[3];

            /* parse "<anything>" for "<lib> : <obj> (<section>)" */
            tokens = rest.match(/(([\w\.\-]*) : )?([\w\.\-]+) \((.+)\)(\s\[.*\])?$/);
            var lib;
            var obj;
            var section;
            if (tokens != null) {
                if (tokens[1] != null) { /* if there is a " : " qualifier */
                    if (tokens[2] != null && tokens[2].length > 0) {
                        /* and if there is a word preceeding the : */
                        lib = tokens[2]; /* set the current library */
                    }
                }
                else {                   /* else there's no lib for this line*/
                    lib = null;
                }
                obj = tokens[3];
                section = tokens[4];
                //print((lib ? ("lib = " + lib + ", ") : "") + "obj = " + obj + ", section = " + section + ", size = " + size);
            }
            else if (rest.match(/\(\.common.+$/)) {
                lib = null;
                obj = null;
                section = rest.substring(1, rest.length - 1);
                //print("section = " + section + ", size = " + size);
            }
            else if (rest.match(/^--HOLE--/)) {
                obj = null;
                section = "--HOLE--";
                //print("section = " + section + ", size = " + size);
            }
            else {
                lib = null;
                obj = null;
                //print("huh?: " + rest);
                continue;
            }

            if (section == "xdc.meta" || section == "xdc.noload") {
                continue;
            }

            /* accumulate sizes for each "container" */
            var key = lib != null ? lib : (obj != null ? obj : ".common+holes");
            if (result[key] == null) {
                result[key] = {name: key, total: 0, sections: {}};
            }
            result[key].total += size;

            /* accumulate sizes of each contained section */
            var ssize = result[key].sections[section];
            result[key].sections[section] = size + (ssize != null ? ssize : 0);
        }
    }

    /* close file stream */
    file.close();

    /* create array of containers and sort by size */
    var carray = [];
    for (var k in result) {
        carray.push(result[k]);
    }
    carray.sort(function (a, b) {return (b.total - a.total);});

    return (carray);
}

/*
 *  ======== symbolValueGnu ========
 *  If line is a symbol definition and it's a symbol of interest, enter it's
 *  value in symbolTable.
 *
 *  We use two hash tables to enable TI and GNU tool chains to use different
 *  names for the same "value"; this enables the display method to be
 *  independent of the toolchain.
 */
function symbolValueGnu(line, symbolNames, symbolTable)
{
    var tokens = line.match(/^\s+(0x[a-fA-F0-9]+)\s+([a-zA-Z_]+[a-zA-Z0-9_+]*)/);
    if (tokens != null && tokens[1] != null && tokens[2] != null) {
        if (symbolNames[tokens[2]] != null) {
            symbolTable[symbolNames[tokens[2]]] = Number(tokens[1]);
            return (true);
        }
    }
    return (false);
}

/*
 *  ======== objectDepsGnu ========
 *  looks for the dependency chains described by line pairs of the form:
 *     library(obj_in_library)
 *         {obj (symbol)|library(obj_in_library) (symbol)}
 *
 *  The "library(obj_in_library)" string can be used as a key to trace 
 *  dependency chains; it's both unique and appears as both an included
 *  archive member and as a "because of" file.
 */
var cam = null;
var depsState = 0;
function objectDepsGnu(line, objectTable)
{
    if (depsState >= 3) {
        return (false);
    }
    else if (line.indexOf("Archive member ") == 0) {
        depsState++;
        return (true);
    }
    else if (line == "") {
        depsState++;
        return (true);
    }

    /* library(obj_in_library) (symbol) */
    var amp = /(([a-zA-Z-0-9_+\\\/:\.]+)\(([a-zA-Z-0-9_+\.]+)\))( \(([a-zA-Z-0-9_+\.:\(\)]+)\))?/;
    var tokens = line.match(amp);
    if (tokens) {
        if (line[0] == ' ') {
            objectTable[cam.name] = {member: cam, referer: tokens[1], symbol: tokens[5]};
        }
        else {
            cam = {name: line, archive: tokens[1], object: tokens[2]};
        }
    }
    else {
        tokens = line.match(/([a-zA-Z-0-9_+\\\/:\.]+) \(([a-zA-Z-0-9_+\.:\(\)]+)\)/);
        if (line[0] != ' ') {
            print("error: " + line);
        }
        else if (tokens != null) {
            objectTable[cam.name] = {member: cam, referer: tokens[1], symbol: tokens[2]};
        }
        else {
            tokens = line.match(/\s+\(([a-zA-Z-0-9_+\.:\(\)]+)\)/);
            if (tokens != null) {
                objectTable[cam.name] = {member: cam, referer: null, symbol: tokens[1]};
            }
        }
    }
    return (true);
}

/*
 *  ======== printDeps ========
 */
function printDeps(ot)
{
    for (var name in ot) {
        var shortName = name.substring(name.lastIndexOf('/') + 1);
        print(shortName + ": is required to define " + ot[name].symbol + " from "
              + ot[name].referer);
    }
}

/*
 *  ======== printDepChains ========
 *  print sequence of objects that leads to the inclusion of each object 
 *  matching the start pattern
 */
function printDepChains(ot, startPattern)
{
    var name = null;
    var i = 0;

    print("\ndependency chains for objects matching '" + startPattern + "':");
    /* look for objects in the object table that match startPattern */
    for (name in ot) {
        if (name.match(startPattern) != null) {
            i++;
            printDepChain(ot, name);
            print("");
        }
    }

    if (name != null && i == 0) {
        print(" no objects match the pattern '" + startPattern + "'");
    }
}

/*
 *  ======== printDepChain ========
 *  Print the dependency chain for the object file whose full name is specified
 */
function printDepChain(ot, fullName)
{
    /* define start to be the canonical shortened fullName */
    var start = fullName.substring(fullName.lastIndexOf('/') + 1);

    /* follow the chain of referers back to an init obj or undefined symbol */
    var prefix = " ";
    var obj = ot[fullName];
    for (var ref = obj.referer; ref; ref = obj.referer) {
        var shortRef = ref.substring(ref.lastIndexOf('/') + 1);
        print(prefix + start + ": required by " + shortRef 
              + " for " + (obj.symbol == null ? "<???>" : obj.symbol));
        prefix += " ";
        start = shortRef;
        obj = ot[ref];
        if (obj == null) {
            print(prefix + ref 
                  + ": required because it's on the command line");
            break;
        }
    }
    if (obj != null) {
        print(prefix + start 
              + ": required to define initially undefined symbol "
              + obj.symbol);
    }
}

/*
 *  ======== findRef ========
 */
function findRef(carray, name)
{
    print("\n  scanning for " + name + " ...");
    if (toolChain != "gnu") {
        print("    can't parse non-gcc map files (yet).");
        return;
    }

    /* create a hash table of all objects linked into the program */
    var objects = {};
    for (var i = 0; i < carray.length; i++) {
        var c = carray[i];

        var isLib = false;
        var objName;
        for (var k in c.sections) {
            var names = k.match(/^([a-zA-Z-0-9_+\.:]+)\(([a-zA-Z-0-9_+\.\\\/]*)\)/);
            if (names && names[2] != null) {
                isLib = true;
                objName = names[2];
                if (objects[objName] == null) {
                    objects[objName] = {name: objName, sections: {}};
                }
                objects[objName].sections[names[1]] = 1;
            }
            else {
                //print("warning: skipped " + k);
            }
        }
        if (!isLib) {
            objName = c.name.split(/\s/)[0];
            objects[objName] = {name: objName, sections: {}};
        }
    }

    /* run nm on all top-level libraries and objects */
    for (var i = 0; i < carray.length; i++) {
        var c = carray[i];
        var tokens = c.name.match(/([a-zA-Z-0-9+_\.]+)\s\(([a-zA-Z-0-9_+\.\\\/]*)\)/);
        if (tokens == null) {
            print("can't parse " + c.name);
            continue;
        }
        /* skip over internal section names */
        if (tokens[1] == "COMMON") {
            continue;
        }

        var prefix = (tokens[2] == "" || tokens[2] == null) ? "." : tokens[2];
        var fileName = prefix + '/' + tokens[1];

        if (java.io.File(fileName).exists()) {

            /* run nm */
            var options = {useEnv: true};
            var result = {};
            xdc.exec(["nm", fileName], options, result);
            if (result.exitStatus != 0) {
                print(result.output);
                java.lang.System.exit(result.exitStatus);
            }

            /* parse the output of nm */
            var header = "  nm " + tokens[1] + " ...";
            var lines = result.output.split(/[\n\r]+/);
            var curObj = tokens[1];
            for (var j = 0; j < lines.length; j++) {
                var line = lines[j];

                /* skip over object file name lines (from "nm <archive>") */
                tokens = line.match(/^([a-zA-Z-0-9_+\.\\\/]*):$/);
                if (tokens && tokens[1]) {
                    curObj = tokens[1];
                    continue;
                }

                /* skip over curObj if it's not in the executable */
                if (objects[curObj] == null) {
                    continue;
                }

                /* if the line matches a reference, print it */
                if (line.match(name)) {
                    if (header != null) {
                        print(header);
                        header = null;
                    }
                    print("    " + curObj + " : " + line + "\n");
                }
            }
        }
        else {
            print("can't find " + fileName + " (" + c.name + ")");
        }
    }
}

/*
 *  ======== computeDiff ========
 */
function computeDiff(baseline, other)
{
    var delta = [];

    /* compute hashtable of baseline container sizes */
    var bsizes = {};
    for (var i = 0; i < baseline.length; i++) {
        var c = baseline[i];
        var name = c.name.replace(/ \(.*\)/, '');
        bsizes[name] = {total: c.total, sections: c.sections};
    }

    /* compute deltas for all containers in other */
    var osizes = {};
    for (var i = 0; i < other.length; i++) {
        var c = other[i];
        var name = c.name.replace(/ \(.*\)/, '');
        osizes[name] = other[i].total;

        delta.push({
            name: name, 
            delta: osizes[name] - (bsizes[name] ? bsizes[name].total : 0),
            sections: {}
        });

        if (verbose) {
            /* if verbose, add list of section differences to delta */
            var d = delta[delta.length - 1];
            if (bsizes[name] && d.delta != 0) {
                /* for each section s in other container c */
                for (var s in c.sections) {
                    /* if s is not in baseline container name */
                    if (!bsizes[name] || !bsizes[name].sections[s]){
                        /* this is an added section */
                        d.sections[s] = c.sections[s];
                    }
                }

                /* for each section s in the baseline container name */
                for (var s in bsizes[name].sections) {
                    /* if s is not in container c */
                    if (!c.sections[s]){
                        /* this is an deleted section */
                        d.sections[s] = 0 - bsizes[name].sections[s];
                    }
                }
            }
        }
    }

    /* add delta's for all containers in baseline _not_ in other */
    for (var name in bsizes) {
        if (!(name in osizes)) {
            delta.push({name: name, delta: 0 - bsizes[name].total});
        }
    }

    /* sort delta array by the magnitude of the difference */
    delta.sort(function(a,b) {return(Math.abs(b.delta) - Math.abs(a.delta));});
    return (delta);
}

/*
 *  ======== display ========
 *  Print the array of values returned by parse()
 */
function display(carray, fileName, verbose)
{
    var pad = "       ";

    /* for each "container", pretty print it, and accumulate their sizes */
    var total = 0;
    for (var i = 0; i < carray.length; i++) {
        var c = carray[i];
        var len = String(c.total).length;
        var name = verbose > 1 ? c.name : c.name.replace(/ \(.*\)/, '');
        print("  " + pad.substring(len) + c.total + " " + name);
        total += c.total;

        if (verbose) {
            /* create a sections array and print it in size order */
            var sections = [];
            for (var k in c.sections) {
                sections.push({name: k, total: c.sections[k]});
            }
            sections.sort(function (a, b) {return (b.total - a.total);});
            var nlines = verbose > 1 ? sections.length : Math.min(5, sections.length);
            for (var j = 0; j < nlines; j++) {
                len = String(sections[j].total).length;
                print("    " + pad.substring(len) + sections[j].total 
                      + " " + sections[j].name);
            }
        }
    }

    var len = String(total).length;
    var suffix = exeAttrs.__TARG__ == null 
        ? "" 
        : (" (" + exeAttrs.__TARG__ + ", " + exeAttrs.__PLAT__ + ")");
    print("  " + pad.substring(len) + total + " TOTAL: " + fileName + suffix);

    var targ = getTargetSuffix(carray);
    
    /* display unused SRAM and FLASH */
    displayMem("SRAM", fileName, targ);
    displayMem("FLASH", fileName, targ);
}

/*
 *  ======== displayDiffs ========
 */
function displayDiffs(basename, sums) 
{
    /* get baseline carray from sums */
    var baseline = sums[basename];

    /* compute delta between baseline and other sums */
    for (var oname in sums) {
        if (oname != basename) {
            /* compute the difference between two maps */
            var delta = computeDiff(baseline, sums[oname]);

            /* print results */
            print("\ndelta: " + oname + " - " + basename);
            var total = 0;
            var len;
            var pad = "         ";
            for (var i = 0; i < delta.length; i++) {
                total += delta[i].delta;

                /* display container delta */
                len = String(delta[i].delta).length;
                print(pad.substring(len) + delta[i].delta
                      + " " + delta[i].name);

                /* convert delta[i].sections into a sortable array */
                var sects = [];
                for (s in delta[i].sections) {
                    var tmp = {name: s, size: delta[i].sections[s]};
                    sects.push(tmp);
                }

                /* sort it */
                sects.sort(function(a,b) {
                               return (Math.abs(b.size) - Math.abs(a.size));
                           });

                /* display delta[i].sections in sorted order */
                for (var j = 0; j < sects.length; j++) {
                    if (verbose < 2 && j >= 5) {
                        break;
                    }
                    var s = sects[j];
                    len = String(s.size).length;
                    print(pad + pad.substring(len) + s.size + " " + s.name);
                }
            }
            len = String(total).length;
            print(pad.substring(len) + total + " TOTAL");
        }
    }
}

/*
 *  ======== displayMem ========
 */
function displayMem(name, fileName, targ)
{
    var pad = "       ";
    targ = (targ == null) ? "" : (" (targ.suffix: " + targ + ")");
    if (exeAttrs.__TARG__ != null) {
        targ = " (" + exeAttrs.__TARG__ 
            + ", " + exeAttrs.__PLAT__ + ")";
    }
    
    /* display unused memory named 'name' */
    var start = symbolTable[name + "_UNUSED_start"];
    var end = symbolTable[name + "_UNUSED_end"];
    var len = symbolTable[name + "_length"];
    if (start != null && end != null) {
        var unused = end - start;
        var suffix = "";
        if (len != null) {
            var percent = len == 0 ? 100 : (unused / len) * 100;
            suffix = " (" + percent.toFixed(0)
                      + "%, used = " + (len - unused) + ")";
        }

        len = String(unused).length;
        print("  " + pad.substring(len) + unused + " UNUSED "
              + name + suffix + ": " + fileName + targ);
    }
}
