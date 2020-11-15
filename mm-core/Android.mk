ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

OMXCORE_CFLAGS += -D_ANDROID_
OMXCORE_CFLAGS += -U_ENABLE_QC_MSG_LOG_
OMXCORE_CFLAGS += -Wno-error -Wno-unused-parameter

#===============================================================================
#             Figure out the targets
#===============================================================================

ifeq ($(TARGET_BOARD_PLATFORM),msm7627a)
MM_CORE_TARGET = 7627A
else ifeq ($(TARGET_BOARD_PLATFORM),msm8660)
MM_CORE_TARGET = 8660
#Comment out following line to disable drm.play component
OMXCORE_CFLAGS += -DENABLE_DRMPLAY
else ifeq ($(TARGET_BOARD_PLATFORM),msm8960)
MM_CORE_TARGET = 8960
else ifeq ($(TARGET_BOARD_PLATFORM),msm8974)
MM_CORE_TARGET = 8974
else
MM_CORE_TARGET = default
endif

#===============================================================================
#             LIBRARY for Android apps
#===============================================================================

LOCAL_C_INCLUDES        := $(LOCAL_PATH)/src/common
LOCAL_C_INCLUDES        += $(LOCAL_PATH)/inc

LOCAL_HEADER_LIBRARIES := \
        libutils_headers

LOCAL_MODULE            := libOmxCore
LOCAL_MODULE_TAGS       := optional
LOCAL_VENDOR_MODULE     := true
LOCAL_SHARED_LIBRARIES  := liblog libdl
LOCAL_CFLAGS            := $(OMXCORE_CFLAGS)

LOCAL_SRC_FILES         := src/common/omx_core_cmp.cpp
LOCAL_SRC_FILES         += src/common/qc_omx_core.c
LOCAL_SRC_FILES         += src/$(MM_CORE_TARGET)/qc_registry_table_android.c

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := media_headers
LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/inc
include $(BUILD_HEADER_LIBRARY)

#===============================================================================
#             LIBRARY for command line test apps
#===============================================================================

include $(CLEAR_VARS)

LOCAL_C_INCLUDES        := $(LOCAL_PATH)/src/common
LOCAL_C_INCLUDES        += $(LOCAL_PATH)/inc

LOCAL_HEADER_LIBRARIES := \
        libutils_headers

LOCAL_MODULE            := libmm-omxcore
LOCAL_MODULE_TAGS       := optional
LOCAL_VENDOR_MODULE     := true
LOCAL_SHARED_LIBRARIES  := liblog libdl
LOCAL_CFLAGS            := $(OMXCORE_CFLAGS)

LOCAL_SRC_FILES         := src/common/omx_core_cmp.cpp
LOCAL_SRC_FILES         += src/common/qc_omx_core.c
LOCAL_SRC_FILES         += src/$(MM_CORE_TARGET)/qc_registry_table.c

include $(BUILD_SHARED_LIBRARY)

endif #BUILD_TINY_ANDROID
