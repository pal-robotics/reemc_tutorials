#!/bin/bash


################################################
# THIS SCRIPT WILL START THE JACK AUDIO DAEMON #
################################################

remotename=$1

bin=`dirname $0`;


ip=`ifconfig tun0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`
echo "listening from $ip";

log="/tmp/jack.log";

#nohup /opt/jack/bin/jackd -S -R -d alsa -d pal_robotics_48 -C -P -S >& /dev/null &
rm -f $log;
nohup ${jackbindir}jackd -R -d alsa  2>&1 | cat >> $log &
echo "waiting for server to start up.";
${jackbindir}jack_wait -w -t 15 2>&1 |  cat >> $log 
echo "launching netmanager";
${jackbindir}jack_load netmanager -i " -a $ip" 2>&1 |  cat >> $log

${jackbindir}jack_wait -t 5
${bin}/jackConnect.sh $remotename


