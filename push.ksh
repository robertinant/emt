#!/bin/ksh

if [ "$PASSWORD" = "" ]; then
    tree="`ls -dr $TREES/emt/emt-b[0-9]* | head -2 | tail -1`"
    if [ -r $tree/.lastword ]; then
	PASSWORD="`cat $tree/.lastword`"
    else
	echo "WARNING: energia push failed."
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
	echo "    cat't find closure archive $base/closure-*.tar.gz"
	exit 0
fi

$base/push.exp $base/msp432/closure.zip msp432-closure.zip \
               $base/cc3200/closure.zip cc3200-closure.zip \
               $base/cc26xx/closure.zip cc26xx-closure.zip \
               $ctar    `basename $ctar`

