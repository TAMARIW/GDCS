#Build tutorial:

```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../rodos/cmake/port/discovery.cmake ..
make -j                
arm-none-eabi-objcopy -O binary tmwstm32 upload.bin    #upload.bin is the the actual program.
```

Now simply plug in the stm32 and copy upload.bin into the drive folder. then unplug and plug in the board to use new program.