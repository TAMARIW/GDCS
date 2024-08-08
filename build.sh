mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/skith.cmake ..
make -j
arm-none-eabi-objcopy -O binary tmwstm32 uploadMeToSTM32.bin
arm-none-eabi-objcopy -O ihex tmwstm32 main.hex