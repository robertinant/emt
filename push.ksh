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

devices="msp432 cc13xx cc3200"
if [ $# -gt 0 ]; then
    devices="$@"
fi

echo "pushing board packages for \"$devices\" to energia.nu ..."
base=./src/bundles/energia
for dev in $devices; do
    dzip="`ls $base/$dev-emt-*.zip 2> /dev/null`"
    if [ -z "$dzip" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find closure archive $base/$dev-emt-*.zip"
	exit 0
    fi
    $base/lpush.ksh $dzip `basename $dzip`
done

echo "pushing energia_17 closures ..."
ctar="`ls $base/closure*.tar.gz 2> /dev/null`"
if [ -z "$ctar" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find 'universal' closure $base/closure-*.tar.gz"
	exit 0
fi

$base/lpush.ksh $base/msp432/closure.zip msp432-closure.zip \
                $base/cc3200/closure.zip cc3200-closure.zip \
                $base/cc26xx/closure.zip cc26xx-closure.zip \
                $base/cc13xx/closure.zip cc13xx-closure.zip \
                $ctar    `basename $ctar`
