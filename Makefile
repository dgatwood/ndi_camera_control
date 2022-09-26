
# On Mac, at least with Homebrew, the cmake command builds x86_64 binaries
# even on arm, so force our binaries to also use that architecture.  Ugh.
UNAME := $(shell uname)

ARCH := $(shell uname -p)

USE_ROCKPI_PINOUTS=1
USE_MRAA=1

ifeq ($(UNAME), Darwin)
CXXFLAGS+=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0 -arch x86_64
LDFLAGS+=-L/usr/local/NDISDK/lib/x64/ -lndi -framework CoreFoundation -framework AppKit -arch x86_64
endif

ifeq ($(UNAME), Linux)
CXXFLAGS+=-I/usr/local/NDISDK/include/ -g -O3
LDFLAGS+=-lndi -ldl -lpthread -lavahi-client -lavahi-common

ifeq ($(ARCH), unknown)
ARCH := $(shell uname -m)
endif

ifeq ($(ARCH), aarch64)
LDFLAGS+=-L/usr/local/NDISDK/lib/aarch64-rpi4-linux-gnueabi
endif
ifeq ($(ARCH), arm)
LDFLAGS+=L/usr/local/NDISDK/lib/arm-rpi4-linux-gnueabihf
endif
ifeq ($(ARCH), i686)
LDFLAGS+=L/usr/local/NDISDK/lib/i686-linux-gnu
endif
ifeq ($(ARCH), x86_64)
LDFLAGS+=L/usr/local/NDISDK/lib/x86_64-linux-gnu
endif

ifeq ($(USE_ROCKPI_PINOUTS), 1)
CXXFLAGS+=-DUSE_ROCKPI_PINOUTS
endif

ifeq ($(USE_MRAA), 1)
CXXFLAGS+=-DUSE_MRAA
LDFLAGS+=-lmraa
else
LDFLAGS+=-lpigpiod_if2
endif

endif


cameracontroller: cameracontroller.cpp LEDConfiguration.h
	${CXX} -std=c++11 cameracontroller.cpp -o cameracontroller ${CXXFLAGS} ${LDFLAGS}

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
