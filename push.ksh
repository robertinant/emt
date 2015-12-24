#!/bin/ksh

if [ "$PASSWORD" = "" ]; then
    echo "WARNING: energia push failed."
    echo "    you must run `/bin/pwd`/push.ksh to push the clousures"
    exit 0
fi

echo "pushing to energia.nu ..."
base=./src/bundles/energia
$base/push.exp $base/msp432/closure.zip msp432-closure.zip \
               $base/cc3200/closure.zip cc3200-closure.zip \
               $base/cc26xx/closure.zip cc26xx-closure.zip \
               ./exports/emt_src.zip    emt_src.zip

