#!/bin/bash
# This script is used to upload the binary to the STM32 board.
# The IP address of the RPI board is to be given as a command line argument.

# Example usage:
# ./upload.sh USB -> Upload the binary to the STM32 board using the connected ST-Link
# ./upload.sh TMW 10.42.0.10 -> Upload the binary to the STM32 board using the RPI board over wifi
# ./upload.sh TMW 10.42.0.10 TMW 192.168.10.2 -> Upload the binary to the STM32 board of the opposite RPI board through the first RPI board over wifi

# Check if the IP address is given as a command line argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 [<USERNAME> <IP address of STM32> (Proxy <USERNAME> <IP address of STM32> )] | [USB]"
    echo "Use USB to upload using the connected ST-Link"
    echo "Use Proxy to upload over the network (e.g. uploading from A to C over B (A->B->C))"
    exit 1
fi

# if a third and fourth argument is given, use them to upload this file and firmware to the rpi and then run this script there 
if [ $# -eq 4 ]; then
    echo "Uploading the binary to the RPI board und then to the STM32 board"
    # Upload the binary to the RPI board. Auto input the password.
    scp -o ProxyJump=$1@$2 ./build/firmware.hex openocd.cfg upload.sh $3@$4:~/bootloader/

    # Connect to the RPI board and run openocd to upload the binary to the STM32 board via simply calling command "openocd"
    ssh -J $1@$2 $3@$4 "cd ~/bootloader && openocd"
    exit 0
fi

# If -l is given as a command line argument, run openocd locally
if [ "$1" == "USB" ]; then
    echo "Uploading the binary to the STM32 board locally"
    openocd -f openocdlocal.cfg
    exit 0
fi


echo "Uploading the binary to the RPI board und then to the STM32 board"
# Upload the binary to the RPI board. Auto input the password.
scp ./build/firmware.hex openocd.cfg $1@$2:~/bootloader/

# Connect to the RPI board and run openocd to upload the binary to the STM32 board via simply calling command "openocd"
ssh $1@$2 "cd ~/bootloader && openocd"