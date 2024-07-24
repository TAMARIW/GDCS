# Build tutorial:

```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../lib/rodos/cmake/port/discovery.cmake ..
make -j                
arm-none-eabi-objcopy -O binary tmwstm32 uploadMeToSTM32.bin
```

Now simply plug in the stm32 and copy uploadMeToSTM32.bin into the stm32 drive folder. then unplug and plug in the board to use new program.