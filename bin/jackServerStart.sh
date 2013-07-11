#!/bin/bash


################################################
# THIS SCRIPT WILL START THE JACK AUDIO DAEMON #
################################################

remotename=$1

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/jack/lib/
bin=`dirname $0`;
#jackbindir="/opt/jack/bin/";


ip=`ifconfig tun0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`

log="/dev/null";
log="/var/log/jack.log";
log="/tmp/jack.log";

#nohup /opt/jack/bin/jackd -S -R -d alsa -d pal_robotics_48 -C -P -S >& /dev/null &
rm -f $log;
nohup ${jackbindir}jackd -R -d alsa -S 2>&1 | cat >> $log &
${jackbindir}jack_wait -w 2>&1 |  cat >> $log 
${jackbindir}jack_load netmanager -i " -a $ip" 2>&1 |  cat >> $log

${jackbindir}jack_wait -t 5
${bin}/jackConnect.sh $remotename


