#!/bin/bash

read -p "Enter Lxplus username: " ACCOUNT

SRCPATH="/home/daq/cerntile_b175sr_ctrl/"
DSTPATH="/eos/experiment/newtile/protoro_fers/cs/table/"
SFSPATH="/home/daq/eos_temp"

echo "Mounting $SFSPATH into $DSTPATH (may fail if already mounted)"
mkdir -p $SFSPATH
sshfs $ACCOUNT"@lxplus.cern.ch:"$DSTPATH $SFSPATH
echo "---"

echo "Starting live sync between local:"
echo "$SRCPATH"
echo "and remote (is it mounted?):"
echo "$SFSPATH (--> $DSTPATH)"
echo "Kill the process to interrupt"
echo "---"

i=0
while true
do

trap "exit" SIGINT

echo "Iteration number $i..."
i=$(($i + 1))

rsync -avz $SRCPATH/* $SFSPATH/. > /dev/null

echo "---"

echo "Done"

echo "---"

sleep 1

done
