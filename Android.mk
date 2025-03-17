# TODO:  Find a better way to separate build configs for ADP vs non-ADP devices
ifneq ($(TARGET_BOARD_AUTO),true)

  ifneq ($(filter msm8996, $(TARGET_BOARD_PLATFORM)),)
    QCOM_MEDIA_ROOT := $(call my-dir)/msm8996
  endif
  ifneq ($(filter msm8998, $(TARGET_BOARD_PLATFORM)),)
    QCOM_MEDIA_ROOT := $(call my-dir)/msm8998
  endif

  ifneq ($(filter msm8996,$(TARGET_BOARD_PLATFORM)),)
    include $(QCOM_MEDIA_ROOT)/mm-video-v4l2/Android.mk
    include $(QCOM_MEDIA_ROOT)/libc2dcolorconvert/Android.mk
  endif

  ifneq ($(filter msm8998,$(TARGET_BOARD_PLATFORM)),)
    include $(QCOM_MEDIA_ROOT)/mm-core/Android.mk
    include $(QCOM_MEDIA_ROOT)/libstagefrighthw/Android.mk
    include $(QCOM_MEDIA_ROOT)/mm-video-v4l2/Android.mk
    include $(QCOM_MEDIA_ROOT)/libc2dcolorconvert/Android.mk
  endif

endif
