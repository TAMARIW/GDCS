
# Check if the port is given as a command line argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <RODOS PORT>"
    exit 1
fi

mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/$1.cmake ..
make -j
cp tmwstm32 firmware.elf
arm-none-eabi-objcopy -O ihex tmwstm32 firmware.hex
