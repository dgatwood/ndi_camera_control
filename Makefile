
CXXFLAGS=-I/usr/local/NDISDK/include/ -g -O0
LDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi -ldl

MACCXXFLAGS=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++
MACLDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi.4



cameracontroller: cameracontroller.cpp

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
