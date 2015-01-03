#!/bin/bash


CHANNELS="point_chan"


# Make channels
eye_ach_mk() {
    for c in $CHANNELS; do
	ach mk -1 -o 666 $c
    done
}

# Remove channels
eye_ach_rm() {
    for c in $CHANNELS; do
	ach rm $c
    done
}

# Start
eye_start() {
    eye_ach_mk
}

# Stop
eye_stop() {
    eye_ach_rm
}

#################
# Selection
#################
case "$1" in
    start)
	eye_start
	;;
    stop)
	eye_stop
	;;
    *)
	echo "Invalid command, Options are start/stop"
	exit 1
	;;
esac
