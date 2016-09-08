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

echo "pushing to energia.nu ..."
base=./src/bundles/energia
ctar="`ls $base/closure*.tar.gz 2> /dev/null`"
if [ -z "$ctar" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find 'universal' closure $base/closure-*.tar.gz"
	exit 0
fi

msp432="`ls $base/msp432-emt-*.zip 2> /dev/null`"
if [ -z "$msp432" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find closure archive $base/msp432-*.zip"
	exit 0
fi

cc3200="`ls $base/cc3200-emt-*.zip 2> /dev/null`"
if [ -z "$cc3200" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find closure archive $base/cc3200-*.zip"
	exit 0
fi

cc13xx="`ls $base/cc13xx-emt-*.zip 2> /dev/null`"
if [ -z "$cc13xx" ]; then
	echo "WARNING: energia push failed."
	echo "    can't find closure archive $base/cc13xx-*.zip"
	exit 0
fi

echo "pushing arduino board packages ..."
$base/lpush.ksh $msp432  `basename $msp432` \
                $cc3200  `basename $cc3200` \
                $cc13xx  `basename $cc13xx`

echo "pushing energia_17 closures ..."
$base/lpush.ksh $base/msp432/closure.zip msp432-closure.zip \
                $base/cc3200/closure.zip cc3200-closure.zip \
                $base/cc26xx/closure.zip cc26xx-closure.zip \
                $base/cc13xx/closure.zip cc13xx-closure.zip \
                $ctar    `basename $ctar`
