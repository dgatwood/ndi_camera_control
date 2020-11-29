
LDFLAGS="-libmpv -libndi -L/usr/local/NDISDK/lib/x64/  -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf"
CFLAGS="-I/usr/local/NDISDK/include/"

cameracontroller: cameracontroller.c

