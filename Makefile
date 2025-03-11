
# On Mac, at least with Homebrew, the cmake command builds x86_64 binaries
# even on arm, so force our binaries to also use that architecture.  Ugh.
UNAME := $(shell uname)

ARCH := $(shell uname -p)

USE_TFBLIB=1
USE_ROCKPI_PINOUTS=0
USE_MRAA=0

ifeq ($(UNAME), Darwin)
CXXFLAGS+=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0 -arch x86_64
LDFLAGS+=-L/usr/local/NDISDK/lib/x64/ -lndi -framework CoreFoundation -framework AppKit -arch x86_64
endif  # Darwin

ifeq ($(UNAME), Linux)
CXXFLAGS+=-I/usr/local/NDISDK/include/ -g -O3
LDFLAGS+=-lndi -ldl -lpthread -lavahi-client -lavahi-common

ifeq ($(ARCH), unknown)
ARCH := $(shell uname -m)
endif  # Unknown arch

ifeq ($(ARCH), aarch64)
LDFLAGS+=-L/usr/local/NDISDK/lib/aarch64-rpi4-linux-gnueabi
endif  # ARM64

ifeq ($(ARCH), arm)
ifeq (,$(wildcard /usr/local/NDISDK/lib/arm-rpi4-linux-gnueabihf))
LDFLAGS+=-L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/
else
LDFLAGS+=-L/usr/local/NDISDK/lib/arm-rpi4-linux-gnueabihf
endif  # Pi 3/4
endif  # ARM

ifeq ($(ARCH), armv7l)
ifeq (,$(wildcard /usr/local/NDISDK/lib/arm-rpi4-linux-gnueabihf))
LDFLAGS+=-L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/
else
LDFLAGS+=-L/usr/local/NDISDK/lib/arm-rpi4-linux-gnueabihf
endif  # Pi 4
endif  # ARMv7l

ifeq ($(ARCH), i686)
LDFLAGS+=-L/usr/local/NDISDK/lib/i686-linux-gnu
endif  # Arch i686

ifeq ($(ARCH), x86_64)
LDFLAGS+=-L/usr/local/NDISDK/lib/x86_64-linux-gnu
endif  # Arch x86-64

ifeq ($(USE_ROCKPI_PINOUTS), 1)
CXXFLAGS+=-DUSE_ROCKPI_PINOUTS
endif  # RockPi

ifeq ($(USE_MRAA), 1)
CXXFLAGS+=-DUSE_MRAA
LDFLAGS+=-lmraa
else
LDFLAGS+=-lpigpiod_if2
endif  # MRAA

ifeq ($(USE_TFBLIB), 1)
CXXFLAGS+=-DUSE_TFBLIB
LDFLAGS+=-ltfb -L./tfblib/build/
else
LDFLAGS+=-lpigpiod_if2
endif

endif  # Linux

cameracontroller: cameracontroller.cpp LEDConfiguration.h
	${CXX} -std=c++11 cameracontroller.cpp -o cameracontroller ${CXXFLAGS} ${LDFLAGS}

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build

libtfb:
	cd tfblib && mkdir build && cd build && cmake ../ && make
