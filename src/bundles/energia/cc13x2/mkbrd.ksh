#!/bin/bash
#
# Create Arduino board package for the core specified by a closure
#
#  Usage: mkbrd <path_to_emt_source_archive> <sdk-installation>
#
#  cc32xx uses the M4 target  => GCC libs install-native/*/lib/armv7e-m
#  msp432 uses the M4F target => GCC libs install-native/*/lib/armv7e-m/fpu
#  cc26xx uses the M3 target  => GCC libs install-native/*/lib/armv7-m
#  cc13xx uses the M3 target  => GCC libs install-native/*/lib/armv7-m
#  cc13x2 uses the M4F target => GCC libs install-native/*/lib/armv7e-m/fpu
#
#  CORE   = the directory name appearing in <closure>/ti/runtime/wiring/<core>
#  SRCDIR = directory containing this file
#
#                        TMPDIR = temp for staging all the zip content
#  <vers>                DSTDIR
#      cores/$CORE       EMTDIR (sources rebuilt for each project)
#      system
#          kernel/tirtos/
#          source/
#          energia/      BUILDDIR (config generated files)
#      variants
#          <board_1>
#              :
#          <board_n>
#
usage="usage: <path_to_emt_source_archive> <sdk-directory>"

# the GNULIB for cc13x2
#GNULIB="armv7-m"

XDCBIN=$TOOLS/vendors/xdc/xdctools_3_50_02_20_core/Linux/bin
if [ ! -d $XDCBIN ]; then
    echo "$0: XDCBIN does not exist: $XDCBIN"
    exit 1
fi

if [ $# -lt 2 ]; then
    echo "Error: Illegal number of parameters"
    echo "$usage"
    exit 1
fi

srczip="$1"
SDK="$2"
if [ ! -r "$srczip" ]; then
    echo "$0: error: emt source archive is not readable: $srczip"
    exit 1
fi
if [ ! -d "$SDK" ]; then
    echo "$0: error: core SDK is not readable: $SDK"
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

# ======== lsp ========
# list all packages (by dot name) in the specified repos
function lsp {
    $XDCBIN/xdcpkg -a -l:%n $*
}

# ======== rmp ========
# remove specified package (by the package's base directory)
function rmp {
    $XDCBIN/xdcrmp $*
}

# ======== rmDups ========
# rm packages from repo ($1) that also appear in the specified repos ($2, ...)
function rmDups {
    top=$1
    shift 1
    chmod -R +w $top
    for repo in $@; do
        # compute dups between $top and $repo
        dups=`lsp $top $repo | sort | uniq -d`
        for p in $dups; do
            echo "removing $p from $top ..."
            # compute package base directory from it's "dot name"
            base="$top/${p//\.//}"
            # remove just this package (but not any nested package)
            rmp $base
        done
    done
}

# compute energia board package semantic version number
vers=`git tag -l 'emt-*' | egrep -o '[a-zA-Z][0-9]+'|tail -1`
patch="`echo $vers | egrep -o '[0-9]+'`"
patch=`expr $patch + 1`
char="`echo $vers | egrep -o '[a-zA-Z]'`"
series=`expr index abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ $char`
SEMVERS="$series.$patch.0"

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

# unzip _all_ emt sources to EMTDIR (= $DSTDIR/cores/{msp432,cc32xx, ...})
echo "unzipping emt sources to $DSTDIR/cores ..."
unzip -q $srczip -d $DSTDIR/cores

# remove sources that are unrelated to the core $CORE
echo "remove unrelated cores ..."
for c in msp432 msp432e cc26xx cc13x2 cc32xx; do
    if [ "$c" != "$CORE" ]; then
	echo "    rm -rf $c ..."
	rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/$c
    fi
done

# remove variant sources (we copy core-specific variants from the closure)
echo "remove board variant sources ..."
rm -rf $DSTDIR/cores/emt/ti/runtime/wiring/$CORE/variants

EMTDIR="$DSTDIR/cores/$CORE"
mv $DSTDIR/cores/emt $EMTDIR

# copy SDK source and kernel to DSTDIR/system
echo "Copying SDK to $DSTDIR/system ..."
mkdir -p "$DSTDIR/system"
cp -r "$SDK/source" "$DSTDIR/system"
mkdir -p "$DSTDIR/system/kernel"
cp -r "$SDK/kernel/tirtos" "$DSTDIR/system/kernel"

# cull SDK (but preserve the packages)
echo "culling ti and iar libraries ..."
chmod -R +w "$DSTDIR/system"
find "$DSTDIR/system" -type d \( -name iar -o -name ccs \) -prune -exec rm -rf {} \;
find "$DSTDIR/system" -type f \( -name "*.aem4" -o -name "*.aem4f" -o -name "*.arm4" -o -name "*.arm4f" -o -name "*.arm3" -o -name "*.aem3" \) -exec rm -f {} \;
rm -rf $DSTDIR/system/kernel/tirtos/packages/ti/targets/omf/elf/docs
rm -rf $DSTDIR/system/kernel/tirtos/packages/ti/targets/arm/rtsarm
rm -rf $DSTDIR/system/kernel/tirtos/packages/ti/sysbios/rom/c28

#echo "culling gcc libc libraries ..."
#gld="`find $DSTDIR/system/kernel -type d -wholename '*/gnu/targets/arm/libs/install-native/arm-none-eabi/lib'`"
#for d in `ls $gld`; do
#    if [ "$d" != "$GNULIB" ]; then
#        echo removing $gld/$d ...
#	rm -rf "$gld/$d"
#    fi
#done

# selectively copy closure files to EMTDIR (inside DSTDIR)
cd $TMPDIR/closure

## remove packages from closure that are in the SDK
rmDups . $DSTDIR/system/kernel/tirtos/packages $DSTDIR/system

## add version file to EMTDIR
VERSIONLINE=$(head -n 1 version.txt)
VERSION=${VERSIONLINE##* }
echo Copying from `/bin/pwd` to $EMTDIR ...
echo "    closure from $VERSION"
cp version.txt $EMTDIR

## copy closure libraries and linker files
echo "Copy libraries and linker scripts"
find . -type f \( -name "*.m3g.lib" -o -name "*.am3g" -o -name "*.lds" \) | cpio -pudm $EMTDIR
find . -type f \( -name "*.m4fg.lib" -o -name "*.am4fg" \) | cpio -pudm $EMTDIR
find . -type f \( -name "*.m4g.lib" -o -name "*.am4g" \) | cpio -pudm $EMTDIR

# remove closure's config generated src dir
rm -rf $EMTDIR/src

## copy closure headers
echo "Copy ti/runtime/wiring headers"
find ./ti/runtime -type f -name "*.h" | cpio -pudm $EMTDIR
find ./ti/drivers/bsp -type f -name "*.h" | cpio -pudm $EMTDIR
echo "Copy xdc headers"
find ./xdc -type f -name "*.h" | cpio -pudm $EMTDIR

## copy board variant sources to appropriate variant directories
echo "Copy board variant sources to Arduino variant directories"
vfiles="`find ./ti/runtime/wiring/*/variants -depth -maxdepth 6 -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" \) -print`"
for f in $vfiles; do
    suffix="`echo $f | egrep -o '/variants/.*'`"
    vdir="$DSTDIR/`dirname $suffix`"
    mkdir -p $vdir
    cp $f $vdir
done

# copy closure's configuro files to BUILDDIR (inside DSTDIR)
BUILDDIR=$DSTDIR/system/energia

## copy configuration generated files
mkdir -p $BUILDDIR
for f in `ls`; do
    if [ -f $f ]; then
	cp $f $BUILDDIR
    fi
done
find ./src -name "*.o" -exec rm -f {} \;
cp -r ./src $BUILDDIR
cp -r ./configPkg $BUILDDIR

## patch linker.cmd to remove libraries that are re-built for every sketch
echo "Filtering linker.cmd to remove libraries rebuilt for each sketch ..."
sed -e 's|\(lib/board.*\)|/* \1 commented out by mkbrd.ksh */|' -e 's|\(ti/runtime/wiring/[a-zA-Z0-9_]*/lib.*\)|/* \1 commented out by mkbrd.ksh */|' linker.cmd > $BUILDDIR/linker.cmd

# create board archive
echo "Creating board package zip archive ..."
rm -f $cwd/$CORE-$VERSION.*
cd $TMPDIR; zip -rq $cwd/$CORE-$VERSION.zip $SEMVERS

# cleanup
chmod -R +w $DSTDIR/system
rm -rf closure $DSTDIR
