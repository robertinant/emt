#!/bin/bash
#
# Create a "universal closure" by merging the closures for multiple 
# "identical except for device" energia configurations and adding emt
# sources.
#
#  Usage: copy <path_to_emt_source_archive>
#
if [ $# -lt 1 ]; then
	echo "Error: Illegal number of parameters"
	echo "usage: ${0##*/} <path to emt source archive>"
	exit 0
fi

HTTP_PROXY="http://wwwgate.ti.com:80"
export http_proxy=$HTTP_PROXY

cwd=$(pwd)

TMPDIR=`mktemp -d -t closureXXX`
echo $TMPDIR
if [ -z "$TMPDIR" ]; then
    echo temp directory could not be created
    exit 1
fi

EMTDIR=$TMPDIR/emt
echo "Creating temporary emt dir: $TMPDIR/emt ..."
if [ $# -ge 1 ]; then
    echo unzipping emt sources to $TMPDIR/emt
    unzip -q $1 -d $TMPDIR
else
    mkdir $EMTDIR
fi

FILES="cc32xx/closure msp432/closure cc26xx/closure cc13xx/closure"

for FILE in $FILES; do
    echo $FILE
    if [ ! -r  "$cwd/$FILE.zip" ]; then
        echo "Failed to get $FILE.zip"
        exit 0
    fi
    mkdir -p $TMPDIR/$FILE/
    unzip -q $cwd/$FILE.zip -d $TMPDIR/$FILE/

    cd $TMPDIR/$FILE/closure
    if [ $? != "0" ]; then
        echo "error: cd to $TMPDIR/$FILE/closure failed"
        exit 1
    fi
    echo copying from `/bin/pwd` ...

    VERSIONLINE=$(head -n 1 version.txt)
    VERSION=${VERSIONLINE##* }
    echo $VERSION

    echo "Copy gnulib libraries ..."
    find ./gnu/targets/arm/libs -type f | cpio -pudm $EMTDIR

    echo "Copy package libraries"
    find . -type f \( -name "*.m3g.lib" -o -name "*.am3g" -o -name "*.lds" \) | cpio -pudm $EMTDIR
    find . -type f \( -name "*.m4fg.lib" -o -name "*.am4fg" \) | cpio -pudm $EMTDIR
    find . -type f \( -name "*.m4g.lib" -o -name "*.am4g" \) | cpio -pudm $EMTDIR
    echo "Copy sysbios headers"
    find ./ti/sysbios -type f -name "*.h" | cpio -pudm $EMTDIR
    echo "Copy drivers headers"
    find ./ti/drivers -type f -name "*.h" | cpio -pudm $EMTDIR
    echo "Copy TI mw headers"
    find ./ti/mw -type f -name "*.h" | cpio -pudm $EMTDIR
    echo "Copy xdc headers"
    find ./xdc -type f -name "*.h" | cpio -pudm $EMTDIR
    echo "Copy gnu headers"
    find ./gnu -type f -name "*.h" | cpio -pudm $EMTDIR
    echo "Copy variants directory"
    find ./ti/runtime/wiring/*/variants -depth -maxdepth 6 -type f \( -name "*.c" -o -name "*.h" \) -print | cpio -pudm $EMTDIR
    echo "Copy linker script and compiler options"
    VDIR=`find ti/runtime/wiring/*/variants -maxdepth 6 -name variants`
    VDIR=$EMTDIR/${VDIR%/*}
    cp linker.cmd $VDIR/
    cp compiler.opt $VDIR/
    cp version.txt $EMTDIR
    echo "Copy configPkg"
    find ./configPkg/package/cfg/ -type f \( -name "*.rov.xs" -o -name "*.h" -o -name "*pm3g.om3g" -o -name "*pm4fg.om4fg" -o -name "*pm4g.om4g" \) | cpio -pudm $VDIR
done

echo "Creating combined closure archive"
cd $EMTDIR
tar -czf $cwd/closure-$VERSION.tar.gz *

rm -r $TMPDIR
