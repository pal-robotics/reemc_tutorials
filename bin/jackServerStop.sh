#!/bin/bash

###############################################
# THIS SCRIPT WILL STOP THE JACK AUDIO DAEMON #
###############################################


killall -9 jackd

rm -f /tmp/jack.log

