#!/bin/bash

################################################
# THIS SCRIPT WILL START THE JACK AUDIO DAEMON #
################################################



ip=$1;

if [[ $ip == "" ]]; then
	echo "Please specify the server IP executing:";
	echo "$0 <ip address>";
	exit;
fi


log="/tmp/jack.log";

asoundrc="$HOME/.asoundrc";
jackasoundrc="/etc/.asoundrc.jack";
if [[ -e $jackasoundrc ]]; then
	cp $jackasoundrc $asoundrc && chmod +w $asoundrc
else
	echo "$jackasoundrc file does not exists, broadcasting is not the default, you can explicitly use the jack device to do so.";
fi

nohup  ${jackbindir}jackd -R -d net -a $ip  >& $log &

