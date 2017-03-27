LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

#===============================================================================
#            Deploy the headers that can be exposed
#===============================================================================

LOCAL_COPY_HEADERS_TO   := mm-video/DivxDrmDecrypt
LOCAL_COPY_HEADERS      := inc/DivXDrmDecrypt.h

LOCAL_CFLAGS := \
    -D_ANDROID_

LOCAL_SRC_FILES:=       \
    src/DivXDrmDecrypt.cpp

LOCAL_C_INCLUDES:= \
    $(LOCAL_PATH)/inc \
    $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_PRELINK_MODULE:= false

LOCAL_MODULE:= libdivxdrmdecrypt
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH_32 := $(TARGET_OUT_VENDOR)/lib
LOCAL_MODULE_PATH_64 := $(TARGET_OUT_VENDOR)/lib64

LOCAL_SHARED_LIBRARIES	:= liblog libdl

LOCAL_LDLIBS +=
include $(BUILD_SHARED_LIBRARY)
