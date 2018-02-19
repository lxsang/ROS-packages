#!/bin/sh
term=$1
while :
do
    ps -p `ps -e | grep "$1" | awk '{print $1}'` -o time,%cpu,%mem >> $HOME/$1.rst
    sleep 5
done
