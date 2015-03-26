Install the driver
==================

lsmod | grep esdcan
cd /usr/src/esdcan-pci405-linux-2.6.x-x86_64-3.9.5/src
ls 
sudo insmod esdcan-pci405.ko
cd /dev
ls
sudo mknod --mode=a+rw can0 c 53 0
sudo mknod --mode=a+rw can1 c 53 1
sudo mknod --mode=a+rw can2 c 53 2
sudo mknod --mode=a+rw can3 c 53 3

Start sns service
==================
sudo service sns start

Create channels
===============
ach -C ref-left
ach -C state-left

ach -C ref-right
ach -C state-right

ach -C gripref-left
ach -C gripstate-left

ach -C gripref-right
ach -C gripstate-right

Start daemons
==============

# Start daemon for left arm
cd ~/Documents/Krang/Software/drivers/pcio-sns/build
./pcio-sns -c ref-left -s state-left -b 0 -m 1 -m 2 -m 3 -m 4 -b 1 -m 5 -m 6 -m 7

# Start daemon for right arm
./pcio-sns -c ref-right -s state-right -b 2 -m 1 -m 2 -m 3 -m 4 -b 3 -m 5 -m 6 -m 7

# Start daemon for both grippers and home them
./pcio-sns -c gripref-left -s gripstate-left -b 1 -m 8 -H
./pcio-sns -c gripref-right -s gripstate-right -b 3 -m 8 -H
