mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/discovery.cmake ..
make -j
arm-none-eabi-objcopy -O binary tmwstm32 uploadMeToSTM32.bin