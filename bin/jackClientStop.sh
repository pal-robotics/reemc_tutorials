#!/bin/bash

###############################################
# THIS SCRIPT WILL STOP THE JACK AUDIO DAEMON #
###############################################

asoundrc="$HOME/.asoundrc";
alsasoundrc="/etc/.asoundrc.alsa";
if [[ -e $alsasoundrc ]]; then
        cp $alsasoundrc $asoundrc && chmod +w $asoundrc 
fi

killall jackd
sleep 1
killall -9 jackd

