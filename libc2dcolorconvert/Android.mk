LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
        C2DColorConverter.cpp

LOCAL_C_INCLUDES := \
    $(TOP)/$(call project-path-for,qcom-display)/libcopybit \
    $(TARGET_OUT_HEADERS)/qcom/display/

LOCAL_SHARED_LIBRARIES := libutils liblog libdl

LOCAL_HEADER_LIBRARIES := generated_kernel_headers
LOCAL_HEADER_LIBRARIES += libstagefright_headers
LOCAL_HEADER_LIBRARIES += media_plugin_headers

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := libc2dcolorconvert

LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
