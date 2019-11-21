ifeq ($(call is-board-platform-in-list, $(QCOM_BOARD_PLATFORMS)),true)

MM_CORE := libmm-omxcore
MM_CORE += libOmxCore
MM_CORE += libplatformconfig

PRODUCT_PACKAGES += $(MM_CORE)

endif

