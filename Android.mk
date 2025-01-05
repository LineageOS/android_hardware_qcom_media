ifeq ($(call my-dir),$(call project-path-for,qcom-media))

QCOM_MEDIA_ROOT := $(call my-dir)

#Compile these for all targets under QCOM_BOARD_PLATFORMS list.
ifeq ($(call is-board-platform-in-list, $(MSM_VIDC_TARGET_LIST)),true)
ifeq ($(TARGET_BOARD_AUTO),true)
include $(QCOM_MEDIA_ROOT)/libsidebandstreamhandle/Android.mk
endif
endif

endif
