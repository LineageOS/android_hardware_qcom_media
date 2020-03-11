ifeq ($(call is-board-platform-in-list, $(QCOM_BOARD_PLATFORMS)),true)

MM_CORE := libmm-omxcore
MM_CORE += libOmxCore
MM_CORE += libplatformconfig
MM_CORE += libcodec2_vndk.vendor
MM_CORE += libcodec2_hidl@1.0.vendor

PRODUCT_PACKAGES += $(MM_CORE)

endif

