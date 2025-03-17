
# On Mac, at least with Homebrew, the cmake command builds x86_64 binaries
# even on arm, so force our binaries to also use that architecture.  Ugh.
UNAME := $(shell uname)

CC=clang
CXX=clang++

ARCH := $(shell uname -p)

USE_TFBLIB=1
USE_ROCKPI_PINOUTS=0
USE_MRAA=0

ifeq ($(UNAME), Darwin)
CXXFLAGS+=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++ -ObjC++ -g -O0 -arch x86_64
LDFLAGS+=-L/usr/local/NDISDK/lib/x64/ -lndi -framework CoreFoundation -framework AppKit -arch x86_64
endif  # Darwin

ifeq ($(UNAME), Linux)
CFLAGS+=-fblocks
CXXFLAGS+=-I/usr/local/NDISDK/include/ -g -O0 -fblocks
LDFLAGS+=-lndi -ldl -lpthread -lavahi-client -lavahi-common -lobjc -lBlocksRuntime
# -I/usr/include/GNUstep -lgnustep-gui

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
DEPS=libtfb
else
DEPS=
endif

endif  # Linux

cameracontroller: cameracontroller.mm LEDConfiguration.h ${DEPS}
	${CXX} -std=c++11 cameracontroller.mm -o cameracontroller ${CXXFLAGS} ${LDFLAGS}

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build

libtfb:
	cd tfblib && mkdir -p build && cd build && cmake ../ && make

libtfb_clone:
	git clone https://github.com/vvaltchev/tfblib.git

install_support:
	sudo apt install clang gnustep gnustep-devel libblocksruntime-dev
	git clone https://github.com/gnustep/libobjc2.git
	# cd libobjc2
	# CC=clang CXX=clang++ cmake .
	# make
	# sudo make install

	# git clone https://github.com/mackyle/blocksruntime.git
	# cd blocksruntime
	# ./buildlib
