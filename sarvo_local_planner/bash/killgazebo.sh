#!/bin/sh

# Define process to find
# proc=$(pidof gzserver)
# # Kills all iperf or command line
# kill -9 $proc


# PID=`ps -ef | grep gzserver | grep -v grep | awk '{print $2}'`
# # if [[ -z "$PID" ]] then
# #   echo "killing $PID"
# #   kill -9 $PID
# # fi

# ps -ef | grep gzserver | grep -v grep | awk '{print $2}' | xargs kill

# Check if gedit is running
# -x flag only match processes whose name (or command line if -f is
# specified) exactly match the pattern. 

if pgrep -x "gzserver" > /dev/null
then
    echo "Killing gzserver"
    ps -ef | grep gzserver | grep -v grep | awk '{print $2}' | xargs kill
else
    echo "No gzserver running"
fi