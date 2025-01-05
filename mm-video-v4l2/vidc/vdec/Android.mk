LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
# 				Common definitons
# ---------------------------------------------------------------------------------

libmm-vdec-def := -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
libmm-vdec-def += -D__align=__alignx
libmm-vdec-def += -Dinline=__inline
libmm-vdec-def += -g -O3
libmm-vdec-def += -DIMAGE_APPS_PROC
libmm-vdec-def += -D_ANDROID_
libmm-vdec-def += -DCDECL
libmm-vdec-def += -DT_ARM
libmm-vdec-def += -DNO_ARM_CLZ
libmm-vdec-def += -UENABLE_DEBUG_LOW
libmm-vdec-def += -UENABLE_DEBUG_HIGH
libmm-vdec-def += -DENABLE_DEBUG_ERROR
libmm-vdec-def += -UINPUT_BUFFER_LOG
libmm-vdec-def += -UOUTPUT_BUFFER_LOG
libmm-vdec-def += -Wno-parentheses
libmm-vdec-def += -D_ANDROID_ICS_
libmm-vdec-def += -D_MSM8974_
libmm-vdec-def += -DPROCESS_EXTRADATA_IN_OUTPUT_PORT
libmm-vdec-def += -DMAX_RES_1080P
libmm-vdec-def += -DMAX_RES_1080P_EBI

TARGETS_THAT_SUPPORT_UBWC := msm8998 sdm660
TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_51 := sdm660
TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_52 := msm8998

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_UBWC)),true)
libmm-vdec-def += -D_UBWC_
endif

libmm-vdec-def += -DUSE_ION

ifeq ($(call is-platform-sdk-version-at-least,27),true) # O-MR1
libmm-vdec-def += -D_ANDROID_O_MR1_DIVX_CHANGES
endif

libmm-vdec-def += -D_QUERY_DISP_RES_

include $(CLEAR_VARS)

# Common Includes
libmm-vdec-inc          := $(LOCAL_PATH)/inc
libmm-vdec-inc          += $(call project-path-for,qcom-media)/mm-video-v4l2/vidc/common/inc
libmm-vdec-inc          += $(call project-path-for,qcom-media)/mm-core/inc
libmm-vdec-inc          += $(TARGET_OUT_HEADERS)/adreno
libmm-vdec-inc          += $(call project-path-for,qcom-media)/libc2dcolorconvert
libmm-vdec-inc          += $(call project-path-for,qcom-media)/hypv-intercept
libmm-vdec-inc          += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

# Common Dependencies
libmm-vdec-add-dep := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

ifeq ($(call is-platform-sdk-version-at-least, 19),true)
# This feature is enabled for Android KK+
libmm-vdec-def += -DADAPTIVE_PLAYBACK_SUPPORTED
endif

ifeq ($(call is-platform-sdk-version-at-least, 22),true)
# This feature is enabled for Android LMR1
libmm-vdec-def += -DFLEXYUV_SUPPORTED
endif

libmm-vdec-def += -DALLOCATE_OUTPUT_NATIVEHANDLE

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_51)),true)
libmm-vdec-def += -DMAX_H264_LEVEL_51
else ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_52)),true)
libmm-vdec-def += -DMAX_H264_LEVEL_52
endif

# Hypervisor
ifneq (,$(filter $(MACHINE), "8x96autogvmquin" "8x96autogvmred"))
libmm-vdec-def += -D_HYPERVISOR_
endif
# ---------------------------------------------------------------------------------
# 			Make the Shared library (libOmxVdec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE                    := libOmxVdec
LOCAL_MODULE_TAGS               := optional
LOCAL_VENDOR_MODULE             := true
LOCAL_CFLAGS                    := $(libmm-vdec-def) -Werror

LOCAL_HEADER_LIBRARIES := \
        media_plugin_headers \
        libnativebase_headers \
        libutils_headers \
        libhardware_headers \
        display_headers

LOCAL_C_INCLUDES                += $(libmm-vdec-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(libmm-vdec-add-dep)

LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := liblog libcutils libdl libqdutils

LOCAL_SHARED_LIBRARIES  += libqdMetaData libhypv_intercept

LOCAL_SRC_FILES         := src/frameparser.cpp
LOCAL_SRC_FILES         += src/h264_utils.cpp
LOCAL_SRC_FILES         += src/ts_parser.cpp
LOCAL_SRC_FILES         += src/mp4_utils.cpp
LOCAL_SRC_FILES         += src/hevc_utils.cpp
LOCAL_STATIC_LIBRARIES  := libOmxVidcCommon
LOCAL_SRC_FILES         += src/omx_vdec_v4l2.cpp

include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------------
#                END
# ---------------------------------------------------------------------------------
