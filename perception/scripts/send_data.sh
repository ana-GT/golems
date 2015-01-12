#!/bin/bash

RGB_CHAN="rgb-chan"
DEPTH_CHAN="depth-chan"
MARKERS_CHAN="markers-chan"

send_data_reset() {
    ach rm $RGB_CHAN
    ach rm $DEPTH_CHAN
}

# Script options
case "$1" in

    # Rgb only
    reset)
	send_data_reset
	;;
    r)
	echo "RGB info sending"
	ach mk -1 -o 666 -m 3 -n 921615 $RGB_CHAN
        echo "Pushing to zhaan"
        achd -rd push zhaan $RGB_CHAN
        echo "Sending program"
	./../../bin/send_data -r $RGB_CHAN
	;;
    rd)
	echo "RGB/Depth info sending"
	ach mk -1 -o 666 -m 3 -n 921615 $RGB_CHAN
	ach mk -1 -o 666 -m 2 -n 3686432 $DEPTH_CHAN
	./../../bin/send_data -r $RGB_CHAN -d $DEPTH_CHAN
	;;
    *)
	echo "[scripts] Invalid command: Valid: r / rd / reset"
	exit 1
	;;
esac
