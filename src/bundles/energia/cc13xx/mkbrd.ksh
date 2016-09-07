#!/bin/bash
#
# Create Arduino board package for the core specified by a closure
#
#  Usage: mkbrd <path_to_emt_source_archive> <ti-rtos_product_tree_name>
#
#  cc3200 uses the M4 target  => GCC libs install-native/*/lib/armv7e-m
#  msp432 uses the M4F target => GCC libs install-native/*/lib/armv7e-m/fpu
#  cc26xx uses the M3 target  => GCC libs install-native/*/lib/armv7-m
#  cc13xx uses the M3 target  => GCC libs install-native/*/lib/armv7-m
#
#  CORE = the directory name appearing in <closure>/ti/runtime/wiring/<core>
#
#                      TMPDIR
#  <vers>              DSTDIR
#      cores/$CORE     EMTDIR
#      system
#          driverlib
#          inc
#      variants
#          <board_1>
#              :
#          <board_n>
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
echo "remove unrelated cores ..."
for c in msp432 cc26xx cc13xx cc3200; do
    if [ "$c" != "$CORE" ]; then
	echo "    rm -rf $c ..."
	rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/$c
    fi
done
mv $DSTDIR/cores/emt $EMTDIR

# copy driverlib library from TI-RTOS product tree to DSTDIR/system
cd $TMPDIR/closure
echo "Copying driverlib to $DSTDIR/system ..."
dlib="`ls -d $TREES/zumaprod/$zptree/exports/tirtos_full_2*/products/cc13xxware_*/`"
mkdir -p "$DSTDIR/system/driverlib/CC13xx"
cp "$dlib"/driverlib/*.[ch] "$DSTDIR/system/driverlib/CC13xx/"
cp -r "$dlib/driverlib/bin/gcc" "$DSTDIR/system/driverlib/CC13xx/"
cp -r "$dlib/inc" "$DSTDIR/system/"

# selectively copy closure files to EMTDIR (inside DSTDIR)
VERSIONLINE=$(head -n 1 version.txt)
VERSION=${VERSIONLINE##* }
echo copying from `/bin/pwd` to $EMTDIR ...
echo "    closure from $VERSION"
cp version.txt $EMTDIR

echo "Copy reentrant gnulib libraries ..."
gnulib=./gnu/targets/arm/libs/install-native/arm-none-eabi
find $gnulib/include -type f | cpio -pudm $EMTDIR
find $gnulib/lib/armv7-m -type f | cpio -pudm $EMTDIR

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

echo "Copy selected configPkg files to ti/runtime/wiring/$CORE"
find ./configPkg/package/cfg/ -type f \( -name "*.rov.xs" -o -name "*.h" -o -name "*pm3g.om3g" -o -name "*pm4fg.om4fg" -o -name "*pm4g.om4g" \) | cpio -pudm $EMTDIR/ti/runtime/wiring/$CORE

# create top-level variant placeholders ...
for v in `ls -d $EMTDIR/ti/runtime/wiring/$CORE/variants/*`; do
    vname=`basename $v`
    mkdir -p $DSTDIR/variants/$vname
    echo "This directory is intensionally empty (almost)" > $DSTDIR/variants/$vname/readme.txt
done

# create board archive
echo "Creating board package zip archive ..."
rm -f $cwd/$CORE-$VERSION.*
cd $TMPDIR; zip -rq $cwd/$CORE-$VERSION.zip $SEMVERS

# cleanup
chmod -R +w $DSTDIR/system
rm -rf closure $DSTDIR
