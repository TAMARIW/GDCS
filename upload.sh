#!/bin/bash
# This script is used to upload the binary to the STM32 board.
# The IP address of the RPI board is to be given as a command line argument.

PASSWORD=zero


# Check if the IP address is given as a command line argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <USERNAME> <IP address of STM32>"
    exit 1
fi

# Upload the binary to the RPI board. Auto input the password.
scp ./build/main.hex openocd.cfg $1@$2:~/bootloader/

# Connect to the RPI board and run openocd to upload the binary to the STM32 board via simply calling command "openocd"
ssh $1@$2 "cd ~/bootloader && openocd"