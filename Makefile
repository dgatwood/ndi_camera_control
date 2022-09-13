
# On Mac, at least with Homebrew, the cmake command builds x86_64 binaries
# even on arm, so force our binaries to also use that architecture.  Ugh.
UNAME := $(shell uname)

USE_MRAA=0

ifeq ($(UNAME), Darwin)
CXXFLAGS=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0 -arch x86_64
LDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi.4 -framework CoreFoundation -framework AppKit -arch x86_64
endif

ifeq ($(UNAME), Linux)
CXXFLAGS=-I/usr/local/NDISDK/include/ -g -O3
LDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi -ldl -lpthread -lavahi-client -lavahi-common
ifeq ($(USE_MRAA), 1)
CFLAGS+=-DUSE_MRAA
LDFLAGS+=-llibmraa
else
LDFLAGS+=-lpigpiod_if2
endif
endif


cameracontroller: cameracontroller.cpp LEDConfiguration.h

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
