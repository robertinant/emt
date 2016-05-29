#!/bin/ksh
#
#  push files to energia.nu via lftp
#
#  Usage: lpush.ksh [local-name remote-name] ...
#
usage="$0 [local-name remote-name] ..."
status=0

#
#  ======== push ========
#  push file $1 to energia as file $2
#
function push {
    echo put "$1" -o "$2" 
    echo put "$1" -o "$2" | lftp -u "energiamt,$PASSWORD" energia.nu
    if [ "$?" != "0" ]; then
        status=1
    fi
}

## push all specified file name pairs to energia
while [ $# -ge 2 ]; do
    push $1 $2
    shift 2
done

## fail if we have an odd number of file names
if [ $# -ge 1 ]; then
    echo $1 not transfered
    echo $usage
    exit 1
fi

exit $status
