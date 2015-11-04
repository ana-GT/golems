#!/bin/bash

CHANNELS="state-left state-right"
CHANNELS="$CHANNELS ref-left ref-right"
CHANNELS="$CHANNELS gripper-left gripper-right"
CHANNELS="$CHANNELS gripperstate-left gripperstate-right"



# Communication channels
TRAJ_CHANNELS="bimanual_chan bimanual_hand_chan"

# Communication channels
COMM_CHANNELS="obj_param_chan"
COMM_CHANNELS="$COMM_CHANNELS module2server_chan"
COMM_CHANNELS="$COMM_CHANNELS server2module_chan"
COMM_CHANNELS="$COMM_CHANNELS gatekeeper_msg_chan"

#**************
# FUNCTIONS
#**************

# Make channels
baxter_ach_mk() {
    for c in $CHANNELS; do
        ach mk -1 -o 666 $c
    done

    # For trajectories
    for c in $TRAJ_CHANNELS; do
        ach mk -m 7 -n 3000 -1 -o 666 $c
    done

    # For comm
    for c in $COMM_CHANNELS; do
        ach mk -m 7 -n 3000 -1 -o 666 $c
    done

}

# Remove channels
baxter_ach_rm() {
    for c in $CHANNELS; do
        ach rm $c
    done

    for c in $TRAJ_CHANNELS; do
        ach rm $c
    done

    # For comm
    for c in $TRAJ_CHANNELS; do
       ach rm $c
    done

}



#***************************
# SET SCRIPT OPTIONS
#***************************
case "$1" in

        #**********************
        # START
        #**********************
        mk)
        baxter_ach_mk
            ;;

        #**********************
        # STOP
        #**********************
        rm)
        baxter_ach_rm
            ;;

        #**********************
        # ALL ELSE
        #**********************
        *)
            echo "[ERROR] Invalid command. Options are mk/rm"   
            exit 1
            ;;
esac



