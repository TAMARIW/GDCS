# Build tutorial:

```
./build.sh <PORT> [CLEAN]
./upload.sh [<USERNAME> <IP address of STM32>] or [USB]
```

Build script must be given which platform (discovery, skith, posix etc.) and giving additionally CLEAN will start the build process from start (Use if problem occur).
Upload script can use used to flash STM32 over WiFi is supported by the raspberry pi or simply use USB to upload using the connected programmer if openocd is installed. 