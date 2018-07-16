LDFLAGS=-g
LDLIBS=-lgio-2.0 -lgobject-2.0 -lm -laravis-0.4 -lnats_static -pthread -lssl -lcrypto -lprotobuf
INCLUDE=-I./ -I/usr/local/include/aravis-0.4/ -I/usr/include/glib-2.0/

driver-me: driver.cc
	g++ $(LDFLAGS) -o driver driver.cc entities.pb.cc libmescan.so.1.0 libllt.so.1.0 $(LDLIBS) $(INCLUDE)
