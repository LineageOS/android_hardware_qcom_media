LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
        C2DColorConverter.cpp

LOCAL_CFLAGS += -Wno-error

LOCAL_HEADER_LIBRARIES := display_headers generated_kernel_headers

LOCAL_SHARED_LIBRARIES := liblog libdl

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := libc2dcolorconvert

include $(BUILD_SHARED_LIBRARY)
