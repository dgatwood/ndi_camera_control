LINUXCXXFLAGS=-I/usr/local/NDISDK/include/ -g -O0
# -O3
LINUXLDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi -ldl -lpthread -lavahi-client -lavahi-common -lpigpiod_if2

MACCXXFLAGS=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0 -arch x86_64
MACLDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi.4 -framework CoreFoundation -framework AppKit -arch x86_64

CXXFLAGS=${LINUXCXXFLAGS}
LDFLAGS=${LINUXLDFLAGS}

# CXXFLAGS=${MACCXXFLAGS}
# LDFLAGS=${MACLDFLAGS}

cameracontroller: cameracontroller.cpp LEDConfiguration.h

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
