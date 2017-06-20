#!/bin/ksh
xdcroot="$1"

# assumes xdcroot is a symlink to a product directory in xdcprod
xdcprod="`cd $xdcroot && /bin/pwd`"
xdcprod="${xdcprod%%/product/*}"
if [ ! -d "$xdcprod" ]; then
    echo "$0: can't determine xdcprod from '$xdcroot'"
    exit 1
fi

# get script that computes imports of a prod tree
imports="$xdcprod/src/scripts/showImports.xs -d 1"

# get top-level platformprod import from xdcprod
$imports $xdcprod | grep platformprod
