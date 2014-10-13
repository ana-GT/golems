#!/bin/bash

#********************************************
# File structure
#********************************************
# 0. Set variables
# 1. Locates $SNS
# 2. Create channels
# 3. Define some useful functions
# 4. Set script options
#********************************************

#************************
# 0. Set variables
#************************

# HANDS: esd CAN Interfaces
CAN_SDH_L=0
CAN_SDH_R=1


#***********************
# 1. LOCATE $SNS
#***********************

# If $SNS string is empty:
if test -z $SNS; then
    # If sns in this path has executable permissions
    if test -x /home/ana/local/etc/init.d/sns; then
	SNS=/home/ana/local/etc/init.d/sns
    elif test -x /usr/local/etc/init.d/sns; then
	SNS=/usr/local/etc/init.d/sns
    elif test -x /etc/init.d/sns; then
	SNS=/etc/init.d/sns
    else
	echo "[ERROR] Could not find SNS program in any of the default locations"
	exit 1
    fi
fi

#*************************
# 2. CREATE CHANNELS
#*************************
CHANNELS="sdhref-left sdhstate-left"
CHANNELS="$CHANNELS sdhref-right sdhstate-right"


#******************************
# 3. DEFINE USEFUL FUNCTIONS
#******************************

# Create channels for arm control
crichton_hands_ach_mk() {
    for c in $CHANNELS; do
	ach mk -1 -o 666 $c
    done 
}

# Remove channels for arm control
crichton_hands_ach_rm() {
    for c in $CHANNELS; do
	ach rm $c
    done
}

# SNS service start
crichton_hands_start_sns() {
    snslogd_is_running=`pgrep snslogd`
	echo "snslogd (if exists) has pid:: ${snslogd_is_running}"
    
    if test -z $snslogd_is_running; then
	echo "\t * Starting SNS service"
	sudo $SNS start
    else 
	echo "\t * SNS already is running"
    fi
}

# Start: Create channels + run daemons
crichton_hands_start() {
    
    crichton_hands_ach_mk
    crichton_hands_start_sns

    # Run sdhiod daemon for SDH hands
    $SNS run -d -r sdh-left -- \
	sdhiod -b $CAN_SDH_L -c sdhref-left -s sdhstate-left
    $SNS run -d -r sdh-right -- \
	sdhiod -b $CAN_SDH_R -c sdhref-right -s sdhstate-right

}

# Expunge: Remove temporal folders for log
crichton_hands_expunge() {
    sudo rm -rf /var/tmp/sns/sdh-left
    sudo rm -rf /var/run/sns/sdh-left

    sudo rm -rf /var/tmp/sns/sdh-right
    sudo rm -rf /var/run/sns/sdh-right
}

# Stop: Stop daemons and programs
crichton_hands_stop() {
    $SNS kill sdh-left
    $SNS kill sdh-right
}

crichton_hands_steal() {
    chown -R $1 /var/run/sns/sdh-left \
	/var/run/sns/sdh-right \
	/var/tmp/sns/sdh-left \
	/var/tmp/sns/sdh-right 
}

#*************************
# 4. Set script options
#**************************

case "$1" in
    start)
	crichton_hands_start
	;;
    stop)
	crichton_hands_stop
	;;
    rm)
	crichton_hands_ach_rm
	;;
    mk) cricthon_hands_ach_mk
	;;
    steal)
	shift
	crichton_hands_steal $@
	;;
    expunge)
	crichton_hands_expunge
	;;
    *)
	echo "[ERROR] Invalid command. Options are start / stop / rm / mk / steal NEW_OWNER / expunge"
	exit 1
	;;
esac
