#!/bin/bash
#
#  Remove all files in a closure that are not needed to use ROV
#
#  Usage : stripc closure-directory
#
usage="$0: usage: stripc closure-dir"
if [ $# -ne 1 ]; then
    echo $usage
    exit 1
fi

if [ ! -d $1 ]; then
    echo "$0: error: $1 not a directory"
    exit 1
fi
if [ ! -d $1/configPkg ]; then
    echo "$0: error: $1 not a closure directory"
    exit 1
fi

op="rm -rf"

## remove well known target-specific directories
find $1 \( -name lib -o -name tests -o -name targets -o -name doc-files -o -name docs \) -exec $op {} \;

## remove target and tool source files
find $1 \( -name "*.c" -o -name "*.cpp" -o -name "*.xdc" -o -name "*.xdt" -o -name "*.mak" -o -name "*.o*" -o -name "*.rel.xml" -o -name "*.h" -o -name "*.java" -o -name "*.vdx" \) -exec $op {} \;

## misc build artifacts
find $1 \( -name "*.cfg.dot" -o -name "*.xdc.inc" -o -name "*.xdc.ninc" -o -name "*.dep" -o -name ".xdcenv.mak" -o -name "package.mak" \) -exec rm -f {} \;

for f in `find $1 -name '*.mak' | egrep '/package/|/lib/'`; do
    echo "    removing $f ..."
    rm -f $f
done

## remove misc non-ROV related files
$op $1/src/sysbios $1/gnu $1/configPkg $1/linker.cmd $1/xdc/bld
$op $1/xdc/shelf/java/ecj.jar $1/xdc/shelf/java/tar.jar
find $1 \( -name "package.bld" \) -exec $op {} \;
find $1/ti -wholename "*/package/cfg" -exec $op {} \;

exit 0
