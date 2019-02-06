# Makefile.am - Automake script for mm-omxvideo
#
ACLOCAL_AMFLAGS = -I m4

BUILD_COMPONENTS := mm-core
BUILD_COMPONENTS += libplatformconfig

if !SKIP_VDEC_BUILD
BUILD_COMPONENTS += libc2dcolorconvert libarbitrarybytes
endif

if BUILD_MM_VIDEO
BUILD_COMPONENTS += mm-video-v4l2
endif

if TARGET_HYPERVISOR
BUILD_COMPONENTS += hypv-intercept
endif

SUBDIRS := $(BUILD_COMPONENTS)
