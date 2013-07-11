#!/bin/bash
PAL_SCRIPT_PATH=`echo $(dirname $(readlink -f ${BASH_SOURCE[0]}))`
. "$PAL_SCRIPT_PATH/palLaunch"
. "$PAL_SCRIPT_PATH/applicationNames"

###############################################
# THIS SCRIPT WILL STOP THE JACK AUDIO DAEMON #
###############################################


bin=`dirname $0`;
$bin/killn.sh jackd

