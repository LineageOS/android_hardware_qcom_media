LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
# 				Common definitons
# ---------------------------------------------------------------------------------

libmm-venc-def := -g -O3 -Dlrintf=_ffix_r
libmm-venc-def += -D__align=__alignx
libmm-venc-def += -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
libmm-venc-def += -DT_ARM
libmm-venc-def += -Dinline=__inline
libmm-venc-def += -D_ANDROID_
libmm-venc-def += -UENABLE_DEBUG_LOW
libmm-venc-def += -UENABLE_DEBUG_HIGH
libmm-venc-def += -DENABLE_DEBUG_ERROR
libmm-venc-def += -UINPUT_BUFFER_LOG
libmm-venc-def += -UOUTPUT_BUFFER_LOG
libmm-venc-def += -USINGLE_ENCODER_INSTANCE
libmm-venc-def += -Werror
libmm-venc-def += -D_ANDROID_ICS_
libmm-venc-def += -D_MSM8974_

TARGETS_THAT_SUPPORT_UBWC := msm8998
TARGETS_THAT_SUPPORT_VQZIP := msm8998
TARGETS_THAT_SUPPORT_PQ :=  msm8998 sdm660
TARGETS_THAT_USE_NV21 := sdm660
TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_51 := sdm660
TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_52 := msm8998
TARGETS_THAT_SUPPORT_LTR := msm8998 sdm660

libmm-venc-def += -DMAX_RES_1080P
libmm-venc-def += -DMAX_RES_1080P_EBI

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_UBWC)),true)
libmm-venc-def += -D_UBWC_
endif

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_USE_NV21)),true)
libmm-venc-def += -D_NV21_
endif

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_VQZIP)),true)
libmm-venc-def += -D_VQZIP_
endif

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_PQ)),true)
libmm-venc-def += -D_PQ_
endif

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_LTR)),true)
libmm-venc-def += -DLTR_SUPPORT
endif

libmm-venc-def += -DUSE_ION
libmm-venc-def += -DUSE_NATIVE_HANDLE_SOURCE
libmm-venc-def += -DSUPPORT_CONFIG_INTRA_REFRESH
libmm-venc-def += -DUSE_CAMERA_METABUFFER_UTILS

# Common Includes
libmm-venc-inc      := $(LOCAL_PATH)/inc
libmm-venc-inc      += $(call project-path-for,qcom-media)/mm-video-v4l2/vidc/common/inc
libmm-venc-inc      += $(call project-path-for,qcom-media)/mm-core/inc
libmm-venc-inc      += $(call project-path-for,qcom-media)/libstagefrighthw
libmm-venc-inc      += $(TARGET_OUT_HEADERS)/adreno
libmm-venc-inc      += $(call project-path-for,qcom-media)/libc2dcolorconvert
libmm-venc-inc      += $(TARGET_OUT_HEADERS)/libvqzip
libmm-venc-inc      += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

# Common Dependencies
libmm-venc-add-dep  := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_51)),true)
libmm-venc-def += -DMAX_H264_LEVEL_51
else ifeq ($(call is-board-platform-in-list, $(TARGETS_THAT_SUPPORT_MAX_H264_LEVEL_52)),true)
libmm-venc-def += -DMAX_H264_LEVEL_52
endif

# ---------------------------------------------------------------------------------
# 			Make the Shared library (libOmxVenc)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE                    := libOmxVenc
LOCAL_MODULE_TAGS               := optional
LOCAL_VENDOR_MODULE             := true
LOCAL_CFLAGS                    := $(libmm-venc-def)

LOCAL_HEADER_LIBRARIES := \
        display_headers \
        media_plugin_headers \
        libnativebase_headers \
        libcutils_headers \
        libutils_headers \
        libhardware_headers \

LOCAL_C_INCLUDES                := $(libmm-venc-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(libmm-venc-add-dep)

LOCAL_SHARED_LIBRARIES    := liblog libcutils libdl

LOCAL_SHARED_LIBRARIES    += libc2dcolorconvert
LOCAL_SHARED_LIBRARIES += libqdMetaData
LOCAL_STATIC_LIBRARIES    := libOmxVidcCommon

LOCAL_SRC_FILES   := src/omx_video_base.cpp
LOCAL_SRC_FILES   += src/omx_video_encoder.cpp
LOCAL_SRC_FILES   += src/video_encoder_device_v4l2.cpp

include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------------
# 					END
# ---------------------------------------------------------------------------------
