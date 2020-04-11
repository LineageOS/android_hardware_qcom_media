AM_CFLAGS = -Wall
AM_CFLAGS += -Wundef
AM_CFLAGS += -Wno-trigraphs
AM_CFLAGS += -g -O3
AM_CFLAGS += "-std=c++11"
AM_CFLAGS += -include stdint.h

AM_CPPFLAGS = $(CPPFLAGS)
AM_CPPFLAGS += -I$(WORKSPACE)/graphics/libdrm/freedreno/kgsl/
AM_CPPFLAGS += -I$(WORKSPACE)/hardware/libhardware/include/

if TARGET_USES_GBM
AM_CPPFLAGS += -DUSE_GBM
endif

c_sources = C2DColorConverter.cpp
h_sources = C2DColorConverter.h

lib_LTLIBRARIES = libc2dcolorconvert.la
libc2dcolorconvert_la_SOURCES = $(c_sources) $(h_sources)
libc2dcolorconvert_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS) -fPIC
libc2dcolorconvert_la_LIBADD = -ldl -lpthread -llog
