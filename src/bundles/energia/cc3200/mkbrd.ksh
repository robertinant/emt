#!/bin/bash
#
# Create Arduino board package for the CC3200
#
#  Usage: mkbrd <path_to_emt_source_archive> <ti-rtos_product_tree_name>
#
usage="usage: <path_to_emt_source_archive> <ti-rtos_product_tree_name>"

if [ $# -lt 2 ]; then
    echo "Error: Illegal number of parameters"
    echo "$usage"
    exit 1
fi

cwd=$(pwd)

SRCDIR=`dirname $0`
SRCDIR=`cd $SRCDIR; /bin/pwd`

TMPDIR="`mktemp -d -t brdpkgXXX`"

if [ -z "$TMPDIR" ]; then
    echo temp directory could not be created
    exit 1
fi
trap "chmod -R +w $TMPDIR;rm -rf $TMPDIR" EXIT INT TERM

zptree=zumaprod-j08
if [ $# -ge 2 ]; then
    zptree="$2"
fi

srczip=../../../../exports/emt_src.zip
if [ $# -ge 1 ]; then
    srczip="$1"
fi

# compute semantic version number
patch="`git tag -l 'emt-*' | egrep -o '[0-9]+'|tail -1`"
patch=`expr $patch + 1`
SEMVERS="1.0.$patch"

# extract closure to TMPDIR
echo "Extracting closure ..."
FILE=closure
if [ ! -r  "$SRCDIR/$FILE.zip" ]; then
    echo "error: $SRCDIR/$FILE.zip is not readable"
    exit 0
fi
unzip -q $SRCDIR/$FILE.zip -d $TMPDIR
if [ $? != "0" ]; then
    echo "error: unzip of closure to $TMPDIR/closure failed"
    exit 1
fi

# create destination directory for final package: DSTDIR (= $TMPDIR/$SEMVERS)
DSTDIR=$TMPDIR/$SEMVERS
mkdir -p $DSTDIR

# copy board and platform files to DSTDIR
cp $SRCDIR/platform.txt $SRCDIR/boards.txt $DSTDIR

# get core name from closure
CORE=`find $TMPDIR/closure/ti/runtime/wiring/*/variants -maxdepth 6 -name variants`
CORE=${CORE%/*}
CORE=${CORE##*/}
echo CORE = $CORE

# unzip _all_ emt sources to EMTDIR (= $DSTDIR/cores/{msp432,cc3200, ...})
echo "unzipping emt sources to $DSTDIR/cores ..."
EMTDIR="$DSTDIR/cores/$CORE"
unzip -q $srczip -d $DSTDIR/cores
rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/msp432
rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/cc26xx
rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/cc13xx
mv $DSTDIR/cores/emt $EMTDIR

# copy driverlib library from TI-RTOS product tree to DSTDIR/system
cd $TMPDIR/closure
echo "Copying driverlib to $DSTDIR/system ..."
dlib="`ls -d $TREES/zumaprod/$zptree/exports/tirtos_full_2*/products/CC32XX_driverlib_*`"
mkdir -p "$DSTDIR/system/driverlib/CC32xx"
cp "$dlib"/driverlib/*.[ch] "$DSTDIR/system/driverlib/CC32xx/"
cp -r "$dlib/driverlib/gcc" "$DSTDIR/system/driverlib/CC32xx/"
cp -r "$dlib/inc" "$DSTDIR/system/"

# selectively copy closure files to EMTDIR (inside DSTDIR)
VERSIONLINE=$(head -n 1 version.txt)
VERSION=${VERSIONLINE##* }
echo copying from `/bin/pwd` to $EMTDIR ...
echo "    closure from $VERSION"
cp version.txt $EMTDIR

#echo "Copy gnulib libraries ..."
#find ./gnu/targets/arm/libs -type f | cpio -pudm $EMTDIR

echo "Copy libraries and linker scripts"
find . -type f \( -name "*.m3g.lib" -o -name "*.am3g" -o -name "*.lds" \) | cpio -pudm $EMTDIR
find . -type f \( -name "*.m4fg.lib" -o -name "*.am4fg" \) | cpio -pudm $EMTDIR
find . -type f \( -name "*.m4g.lib" -o -name "*.am4g" \) | cpio -pudm $EMTDIR

echo "Copy TI-RTOS headers"
find ./ti/sysbios -type f -name "*.h" | cpio -pudm $EMTDIR
find ./ti/drivers -type f -name "*.h" | cpio -pudm $EMTDIR
find ./ti/mw -type f -name "*.h" | cpio -pudm $EMTDIR
find ./xdc -type f -name "*.h" | cpio -pudm $EMTDIR
find ./gnu -type f -name "*.h" | cpio -pudm $EMTDIR

echo "Copy variants directories"
find ./ti/runtime/wiring/*/variants -depth -maxdepth 6 -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" \) -print | cpio -pudm $EMTDIR

echo "Copy linker script and compiler options to ti/runtime/wiring/$CORE"
cp linker.cmd compiler.opt $EMTDIR/ti/runtime/wiring/$CORE

echo "Copy configPkg files to ti/runtime/wiring/$CORE"
find ./configPkg/package/cfg/ -type f \( -name "*.rov.xs" -o -name "*.h" -o -name "*pm3g.om3g" -o -name "*pm4fg.om4fg" -o -name "*pm4g.om4g" \) | cpio -pudm $EMTDIR/ti/runtime/wiring/$CORE

# create board archive and cleanup
echo "Creating board package archive"
cd $TMPDIR
mkdir -p $SEMVERS/variants/CC3200_LAUNCHXL
echo "This directory is intensionally empty (almost)" > $SEMVERS/variants/CC3200_LAUNCHXL/readme.txt
rm -f $cwd/$CORE-$VERSION.*
zip -rq $cwd/$CORE-$VERSION.zip $SEMVERS
