#!/bin/bash

###############################################
# THIS SCRIPT WILL STOP THE JACK AUDIO DAEMON #
###############################################

killall jackd
sleep 1
killall -9 jackd

rm -f /tmp/jack.log

