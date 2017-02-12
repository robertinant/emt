#!/bin/ksh

if [ -r ./.lastword ]; then
    export PASSWORD="`cat ./.lastword`"
fi

if [ "$PASSWORD" = "" ]; then
    tree="`ls -dr $TREES/emt/emt-b[0-9][0-9]* | egrep -v '@|.*x$' | head -2 | tail -1`"
    if [ -r $tree/.lastword ]; then
	export PASSWORD="`cat $tree/.lastword`"
    else
	echo "WARNING: energia push failed: can't read $tree"
	echo "    you must run `/bin/pwd`/push.ksh to push the closures"
	exit 0
    fi
    touch ./.lastword; chmod go-r ./.lastword
    echo "$PASSWORD" > ./.lastword
fi

devices="msp432 cc13xx cc32xx"
if [ $# -gt 0 ]; then
    devices="$@"
fi

echo "pushing board packages for \"$devices\" to energia.nu ..."
base=./src/bundles/energia
for dev in $devices; do
    dzip="`ls $base/$dev-emt-*.zip 2> /dev/null`"
    if [ -z "$dzip" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find board package $base/$dev-emt-*.zip"
    else
	$base/lpush.ksh $dzip `basename $dzip`
    fi
done
