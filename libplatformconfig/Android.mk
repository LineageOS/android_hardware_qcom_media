LOCAL_PATH := $(call my-dir)
LOCAL_DIR_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_COPY_HEADERS_TO := libplatformconfig

libplatformconfig-def := \
      -g0 -O3 -fpic \
      -Wno-deprecated-declarations -Werror \
        -Wno-error=unused-variable \
        -w -Wall -Wextra\
    -fexceptions \
    -Wno-missing-field-initializers \
    -D_ANDROID_

COMMON_CFLAGS := -O3

include $(BUILD_COPY_HEADERS)

LOCAL_CFLAGS := $(COMMON_CFLAGS) $(libplatformconfig-def)

LOCAL_SHARED_LIBRARIES += \
            libexpat \
            liblog \
            libcutils \
            libutils

LOCAL_STATIC_LIBRARIES := libOmxVidcCommon

LOCAL_C_INCLUDES += \
            external/expat/lib \
            $(LOCAL_PATH)/../mm-core/inc \
            $(LOCAL_PATH)/../mm-video-v4l2/vidc/common/inc/ \

LOCAL_SRC_FILES := PlatformConfig.cpp
LOCAL_SRC_FILES += ConfigParser.cpp

LOCAL_MODULE := libplatformconfig
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
