
LDFLAGS=-L/usr/local/NDISDK/lib/x64/ -L/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf -lndi.4
CXXFLAGS=-I/usr/local/NDISDK/include/ -std=c++11 -stdlib=libc++

cameracontroller: cameracontroller.cpp

libmpv:
	cd mpv && ./bootstrap.py  && ./waf configure --enable-libmpv-static --enable-lgpl && ./waf build
