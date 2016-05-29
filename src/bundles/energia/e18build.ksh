#!/bin/ksh
#
#  Command line build for Energia 18
#
#  Usage: e18build <sketch> <core> [<build_output>]
#
#  Example: To build MultiBlink.ino for the MSP432 MSP-EXP430F5529LP and 
#  place all output in the ./build directory
#
#    e18build tests/MultiBlink/MultiBlink.ino energia:msp432:MSP-EXP432P401R
#
#  Note: This script assumes a specific "product" folder structure illustrated
#        by the initial MSP432 EXP432P401R launchpad supported:
#            <root>/packages/energia/hardware/msp432/1.0.13
#
#       ./gold came from Robert's 
#           http://146.252.162.220/energia-cmd-with-msp430-msp432.tar.bz2 

usage="usage: $0 <sketch> <core> [<build_output>]"

## location of the Energia 18 product installation
base="./gold/energia-cmd"
if [ ! -f $base/arduino-builder ]; then
    echo "error: installation root Energia 18 isn't set in the script $0"
    exit 1
fi

## basic command line option checks
if [ $# -lt 2 -o ! -r $1 ]; then
    echo $usage
    exit 1
fi

## set output directory based on optional arguments
build="$(pwd)/build"
if [ $# -gt 2 ]; then
    build="$3"
fi
mkdir -p "$build"

## run the build
$base/arduino-builder -compile -logger=machine -hardware "$base/packages" -tools "$base/tools-builder" -tools "$base/packages" -libraries "$base/libraries" -fqbn=$2 -ide-version=10609 -build-path "$build" -warnings=default -verbose "$1"
