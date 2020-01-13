CONFIG_PATH := hardware/qcom/media/conf_files/$(MSMSTEPPE)

# Video feature flags

# Video configuration files
PRODUCT_COPY_FILES += \
    $(CONFIG_PATH)/codec2.vendor.ext.policy:$(TARGET_COPY_OUT_VENDOR)/etc/seccomp_policy/codec2.vendor.ext.policy \
    $(CONFIG_PATH)/media_codecs.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs.xml \
    $(CONFIG_PATH)/media_codecs_performance.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_performance.xml \
    $(CONFIG_PATH)/media_codecs_vendor.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_vendor.xml \
    $(CONFIG_PATH)/media_codecs_vendor_audio.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_vendor_audio.xml \
    $(CONFIG_PATH)/media_codecs_sdmmagpie_v0.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_v1.xml \
    $(CONFIG_PATH)/media_codecs_sdmmagpie_v1.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_v2.xml \
    $(CONFIG_PATH)/media_codecs_vendor_sdmmagpie_v0.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_vendor_v1.xml \
    $(CONFIG_PATH)/media_codecs_vendor_sdmmagpie_v1.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_vendor_v2.xml \
    $(CONFIG_PATH)/media_codecs_performance_sdmmagpie_v0.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_performance_v1.xml \
    $(CONFIG_PATH)/media_codecs_performance_sdmmagpie_v1.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_codecs_performance_v2.xml \
    $(CONFIG_PATH)/media_profiles.xml:$(TARGET_COPY_OUT_VENDOR)/etc/media_profiles_vendor.xml \
    $(CONFIG_PATH)/mediacodec-seccomp.policy:$(TARGET_COPY_OUT_VENDOR)/etc/seccomp_policy/mediacodec.policy \
    $(CONFIG_PATH)/system_properties.xml:$(TARGET_COPY_OUT_VENDOR)/etc/system_properties.xml

# Produce packages addition
PRODUCT_PACKAGES += \
    libcodec2_vndk.vendor \
    libcodec2_hidl@1.0.vendor

ifneq (,$(filter true, $(TARGET_FWK_SUPPORTS_FULL_VALUEADDS)$(TARGET_BOARD_AUTO)))
  $(warning "Compiling with full value-added framework or for AUTO Platform")
else
  $(warning "Compiling without full value-added framework - enabling GENERIC_ODM_IMAGE")
  GENERIC_ODM_IMAGE := true
endif

# Vendor property overrides

ifeq ($(GENERIC_ODM_IMAGE),true)
  $(warning "Forcing codec2.0 HW for generic odm build variant")
  #Set default ranks and rank Codec 2.0 over OMX codecs
  PRODUCT_ODM_PROPERTIES += debug.stagefright.ccodec=4
  PRODUCT_ODM_PROPERTIES += debug.stagefright.omx_default_rank=1000
else
  $(warning "Enabling codec2.0 SW only for non-generic odm build variant")
  #Rank OMX SW codecs lower than OMX HW codecs
  PRODUCT_PROPERTY_OVERRIDES += debug.stagefright.omx_default_rank.sw-audio=1
  PRODUCT_PROPERTY_OVERRIDES += debug.stagefright.omx_default_rank=0
endif

# Enable Codec2.0 HAL for pure AOSP variants.
DEVICE_MANIFEST_FILE += hardware/qcom/media/conf_files/$(MSMSTEPPE)/c2_manifest.xml
