#!/bin/bash

set -i

if [[ $# -ne 1 ]]; then
	echo "use $0 <robothostname>";
fi

if [[ $1 != "" ]]; then
	clientname=$1;
fi

jack_connect system:capture_1 ${clientname}:to_slave_1
jack_connect system:capture_2 ${clientname}:to_slave_2
jack_connect ${clientname}:from_slave_1 system:playback_1 
jack_connect ${clientname}:from_slave_2 system:playback_2


