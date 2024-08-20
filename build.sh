
# Check if the port is given as a command line argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <RODOS PORT> [CLEAN]"
    exit 1
fi

# if CLEAN is given as a command line argument, clean the build directory
if [ "$2" == "CLEAN" ]; then
    rm -rf build
fi

mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/$1.cmake ..
make -j
cp tmwstm32 firmware.elf
arm-none-eabi-objcopy -O ihex tmwstm32 firmware.hex
