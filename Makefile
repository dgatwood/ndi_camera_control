LINUXCXXFLAGS=-I/usr/local/NDISDK/include/ -g -O3
LINUXLDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi -ldl -lpthread -lavahi-client -lavahi-common

MACCXXFLAGS=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0
MACLDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi.4 -framework CoreFoundation -framework AppKit

CXXFLAGS=${LINUXCXXFLAGS}
LDFLAGS=${LINUXLDFLAGS}

# CXXFLAGS=${MACCXXFLAGS}
LDFLAGS=${MACLDFLAGS}

cameracontroller: cameracontroller.cpp

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
