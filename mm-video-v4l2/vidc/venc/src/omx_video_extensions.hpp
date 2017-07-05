/*--------------------------------------------------------------------------
Copyright (c) 2017, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/

void omx_video::init_vendor_extensions(VendorExtensionStore &store) {

    //TODO: add extensions based on Codec, m_platform and/or other capability queries

    ADD_EXTENSION("qti-ext-enc-preprocess-rotate", OMX_IndexConfigCommonRotate, OMX_DirOutput)
    ADD_PARAM_END("angle", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-avc-intra-period", OMX_IndexConfigVideoAVCIntraPeriod, OMX_DirOutput)
    ADD_PARAM    ("n-pframes",    OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("n-idr-period", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-error-correction", OMX_IndexParamVideoErrorCorrection, OMX_DirOutput)
    ADD_PARAM_END("resync-marker-spacing-bits", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-custom-profile-level", OMX_IndexParamVideoProfileLevelCurrent, OMX_DirOutput)
    ADD_PARAM    ("profile", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("level",   OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-timestamp-source-avtimer", OMX_QTIIndexParamEnableAVTimerTimestamps, OMX_DirInput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr-count", OMX_QcomIndexParamVideoLTRCount, OMX_DirInput)
    ADD_PARAM_END("num-ltr-frames", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr", OMX_QcomIndexConfigVideoLTRUse, OMX_DirInput)
    ADD_PARAM_END("use-frame", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr", OMX_QcomIndexConfigVideoLTRMark, OMX_DirInput)
    ADD_PARAM_END("mark-frame", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-sar", OMX_QcomIndexParamVencAspectRatio, OMX_DirInput)
    ADD_PARAM    ("width", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("height", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-extradata-enable", OMX_QcomIndexParamIndexExtraDataType, OMX_DirOutput)
    ADD_PARAM_END("types", OMX_AndroidVendorValueString)

}

OMX_ERRORTYPE omx_video::get_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext) {
    if (ext->nIndex >= mVendorExtensionStore.size()) {
        return OMX_ErrorNoMore;
    }

    const VendorExtension& vExt = mVendorExtensionStore[ext->nIndex];
    DEBUG_PRINT_LOW("VendorExt: getConfig: index=%u (%s)", ext->nIndex, vExt.name());

    vExt.copyInfoTo(ext);
    if (ext->nParamSizeUsed < vExt.paramCount()) {
        // this happens during initial getConfig to query only extension-name and param-count
        return OMX_ErrorNone;
    }

    // We now have sufficient params allocated in extension data passed.
    // Following code is to set the extension-specific data

    bool setStatus = true;

    switch ((OMX_U32)vExt.extensionIndex()) {
        case OMX_IndexConfigCommonRotate:
        {
            setStatus &= vExt.setParamInt32(ext, "angle", m_sConfigFrameRotation.nRotation);
            break;
        }
        case OMX_IndexConfigVideoAVCIntraPeriod:
        {
            setStatus &= vExt.setParamInt32(ext, "n-pframes", m_sConfigAVCIDRPeriod.nPFrames);
            setStatus &= vExt.setParamInt32(ext, "n-idr-period", m_sConfigAVCIDRPeriod.nIDRPeriod);
            break;
        }
        case OMX_IndexParamVideoErrorCorrection:
        {
            // "bits" @0
            setStatus &= vExt.setParamInt32(ext,
                    "resync-marker-spacing-bits", m_sErrorCorrection.nResynchMarkerSpacing);
            break;
        }
        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            setStatus &= vExt.setParamInt32(ext, "profile", m_sParamProfileLevel.eProfile);
            setStatus &= vExt.setParamInt32(ext, "level", m_sParamProfileLevel.eLevel);

            break;
        }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamAVTimerTimestampMode.bEnable);
            break;
        }
        case OMX_QcomIndexParamVideoLTRCount:
        {
            setStatus &= vExt.setParamInt32(ext, "num-ltr-frames", m_sParamLTRCount.nCount);
            break;
        }
        case OMX_QcomIndexConfigVideoLTRUse:
        {
            setStatus &= vExt.setParamInt32(ext, "use-frame", m_sConfigLTRUse.nID);
            break;
        }
        case  OMX_QcomIndexConfigVideoLTRMark:
        {
            break;
        }
        case  OMX_QcomIndexParamVencAspectRatio:
        {
            setStatus &= vExt.setParamInt32(ext, "width", m_sSar.nSARWidth);
            setStatus &= vExt.setParamInt32(ext, "height", m_sSar.nSARHeight);
            break;
        }
        case  OMX_QcomIndexParamIndexExtraDataType:
        {
            char exType[OMX_MAX_STRINGVALUE_SIZE+1];
            memset (exType,0, (sizeof(char)*OMX_MAX_STRINGVALUE_SIZE));
            if ((OMX_BOOL)(m_sExtraData & VEN_EXTRADATA_LTRINFO)){
                if((strlcat(exType, getStringForExtradataType(OMX_ExtraDataVideoLTRInfo),
                                OMX_MAX_STRINGVALUE_SIZE)) >= OMX_MAX_STRINGVALUE_SIZE) {
                    DEBUG_PRINT_LOW("extradata string size exceeds size %d",OMX_MAX_STRINGVALUE_SIZE );
                }
            }
            if ((OMX_BOOL)(m_sExtraData & VENC_EXTRADATA_MBINFO)) {
                if (exType[0]!=0) {
                    strlcat(exType,"|", OMX_MAX_STRINGVALUE_SIZE);
                }
                if((strlcat(exType, getStringForExtradataType(OMX_ExtraDataVideoEncoderMBInfo),
                                OMX_MAX_STRINGVALUE_SIZE)) >= OMX_MAX_STRINGVALUE_SIZE) {
                    DEBUG_PRINT_LOW("extradata string size exceeds size %d",OMX_MAX_STRINGVALUE_SIZE );
                }
            }
            setStatus &= vExt.setParamString(ext, "types", exType);
            DEBUG_PRINT_LOW("VendorExt: getparam: Extradata %s",exType);
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }
    return setStatus ? OMX_ErrorNone : OMX_ErrorUnsupportedSetting;
}

OMX_ERRORTYPE omx_video::set_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext) {
    if (ext->nIndex >= mVendorExtensionStore.size()) {
        DEBUG_PRINT_ERROR("unrecognized vendor extension index (%u) max(%u)",
                ext->nIndex, mVendorExtensionStore.size());
        return OMX_ErrorBadParameter;
    }

    const VendorExtension& vExt = mVendorExtensionStore[ext->nIndex];
    DEBUG_PRINT_LOW("VendorExt: setConfig: index=%u (%s)", ext->nIndex, vExt.name());

    OMX_ERRORTYPE err = OMX_ErrorNone;
    err = vExt.isConfigValid(ext);
    if (err != OMX_ErrorNone) {
        return OMX_ErrorUnsupportedSetting;
    }

    // mark this as set, regardless of set_config succeeding/failing.
    // App will know by inconsistent values in output-format
    vExt.set();

    bool valueSet = false;
    switch ((OMX_U32)vExt.extensionIndex()) {
        case OMX_IndexConfigCommonRotate:
        {
            OMX_CONFIG_ROTATIONTYPE rotationParam;
            memcpy(&rotationParam, &m_sConfigFrameRotation, sizeof(OMX_CONFIG_ROTATIONTYPE));
            valueSet |= vExt.readParamInt32(ext, "angle", &rotationParam.nRotation);
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_IndexConfigCommonRotate : %d",
                    rotationParam.nRotation);

            err = set_config(
                    NULL, OMX_IndexConfigCommonRotate, &rotationParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigCommonRotate failed !");
            }
            break;
        }
        case OMX_IndexConfigVideoAVCIntraPeriod:
        {
            OMX_VIDEO_CONFIG_AVCINTRAPERIOD idrConfig;
            memcpy(&idrConfig, &m_sConfigAVCIDRPeriod, sizeof(OMX_VIDEO_CONFIG_AVCINTRAPERIOD));
            valueSet |= vExt.readParamInt32(ext, "n-pframes", (OMX_S32 *)&(idrConfig.nPFrames));
            valueSet |= vExt.readParamInt32(ext, "n-idr-period", (OMX_S32 *)&(idrConfig.nIDRPeriod));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: AVC-intra-period : nP=%d, nIDR=%d",
                    idrConfig.nPFrames, idrConfig.nIDRPeriod);

            err = set_config(
                    NULL, OMX_IndexConfigVideoAVCIntraPeriod, &idrConfig);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigVideoAVCIntraPeriod failed !");
            }
            break;
        }
        case OMX_IndexParamVideoErrorCorrection:
        {
            OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE ecParam;
            memcpy(&ecParam, &m_sErrorCorrection, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
            valueSet |= vExt.readParamInt32(ext,
                    "resync-marker-spacing-bits", (OMX_S32 *)&(ecParam.nResynchMarkerSpacing));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: resync-marker-spacing : %d bits",
                    ecParam.nResynchMarkerSpacing);

            err = set_parameter(
                    NULL, OMX_IndexParamVideoErrorCorrection, &ecParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexParamVideoErrorCorrection failed !");
            }
            break;
        }
        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            OMX_VIDEO_PARAM_PROFILELEVELTYPE profileParam;
            memcpy(&profileParam, &m_sParamProfileLevel, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
            valueSet |= vExt.readParamInt32(ext, "profile", (OMX_S32 *)&(profileParam.eProfile));
            valueSet |= vExt.readParamInt32(ext, "level", (OMX_S32 *)&(profileParam.eLevel));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: custom-profile/level : profile=%u level=%u",
                    (OMX_U32)profileParam.eProfile, (OMX_U32)profileParam.eLevel);

            err = set_parameter(
                    NULL, OMX_IndexParamVideoProfileLevelCurrent, &profileParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexParamVideoProfileLevelCurrent failed !");
            }

            break;
        }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
        {
            QOMX_ENABLETYPE avTimerEnableParam;
            memcpy(&avTimerEnableParam, &m_sParamAVTimerTimestampMode, sizeof(QOMX_ENABLETYPE));
            valueSet |= vExt.readParamInt32(ext, "enable", (OMX_S32 *)&(avTimerEnableParam.bEnable));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: AV-timer timestamp mode enable=%u", avTimerEnableParam.bEnable);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamEnableAVTimerTimestamps, &avTimerEnableParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamEnableAVTimerTimestamps failed !");
            }

            break;
        }
        case OMX_QcomIndexParamVideoLTRCount:
        {
            QOMX_VIDEO_PARAM_LTRCOUNT_TYPE ltrCountParam;
            memcpy(&ltrCountParam, &m_sParamLTRCount, sizeof(QOMX_VIDEO_PARAM_LTRCOUNT_TYPE));
            valueSet |= vExt.readParamInt32(ext, "num-ltr-frames",
                                 (OMX_S32 *)&(ltrCountParam.nCount));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: LTR count =%d ", ltrCountParam.nCount);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexParamVideoLTRCount, &ltrCountParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexParamVideoLTRCount failed !");
            }
            break;
        }
        case OMX_QcomIndexConfigVideoLTRUse:
        {
            QOMX_VIDEO_CONFIG_LTRUSE_TYPE ltrUseParam;
            OMX_INIT_STRUCT(&ltrUseParam, QOMX_VIDEO_CONFIG_LTRUSE_TYPE);
            ltrUseParam.nPortIndex = (OMX_U32)PORT_INDEX_IN;
            valueSet |= vExt.readParamInt32(ext, "use-frame", (OMX_S32 *)&(ltrUseParam.nID));
            if (!valueSet) {
                break;
            }
            DEBUG_PRINT_HIGH("VENDOR-EXT: LTR UseFrame id= %d ", ltrUseParam.nID);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRUse, &ltrUseParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexConfigVideoLTRUse failed !");
            }
            break;
        }
        case OMX_QcomIndexConfigVideoLTRMark:
        {
            QOMX_VIDEO_CONFIG_LTRMARK_TYPE ltrMarkParam;
            OMX_INIT_STRUCT(&ltrMarkParam, QOMX_VIDEO_CONFIG_LTRMARK_TYPE);
            ltrMarkParam.nPortIndex = (OMX_U32)PORT_INDEX_IN;
            valueSet |= vExt.readParamInt32(ext, "mark-frame", (OMX_S32 *)&(ltrMarkParam.nID));
            if (!valueSet) {
                break;
            }
            DEBUG_PRINT_HIGH("VENDOR-EXT: LTR mark frame =%d ", ltrMarkParam.nID);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRMark, &ltrMarkParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexConfigVideoLTRMark failed !");
            }

            break;
        }
        case  OMX_QcomIndexParamVencAspectRatio:
        {
            QOMX_EXTNINDEX_VIDEO_VENC_SAR aspectRatioParam;
            OMX_INIT_STRUCT(&aspectRatioParam, QOMX_EXTNINDEX_VIDEO_VENC_SAR);
            valueSet |= vExt.readParamInt32(ext, "width", (OMX_S32 *)&(aspectRatioParam.nSARWidth));
            valueSet |= vExt.readParamInt32(ext, "height", (OMX_S32 *)&(aspectRatioParam.nSARHeight));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: VencAspectRatio: width =%d, height=%d",
              aspectRatioParam.nSARWidth, aspectRatioParam.nSARHeight);
            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVencAspectRatio, &aspectRatioParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamVencAspectRatio failed !");
            }
            break;
        }
        case  OMX_QcomIndexParamIndexExtraDataType:
        {
            QOMX_INDEXEXTRADATATYPE extraDataParam;
            char exType[OMX_MAX_STRINGVALUE_SIZE];
            OMX_INIT_STRUCT(&extraDataParam, QOMX_INDEXEXTRADATATYPE);
            valueSet |= vExt.readParamInt64(ext, "types", exType);
            if (!valueSet) {
                break;
            }
            char *rest = exType;
            char *token = strtok_r(exType, "|", &rest);
            do {
                extraDataParam.bEnabled = OMX_TRUE;
                extraDataParam.nIndex = (OMX_INDEXTYPE)getIndexForExtradataType(token);
                if (extraDataParam.nIndex < 0) {
                    DEBUG_PRINT_HIGH(" extradata %s not supported ",token);
                    continue;
                }
                if (extraDataParam.nIndex == (OMX_INDEXTYPE)OMX_ExtraDataVideoLTRInfo ||
                    extraDataParam.nIndex == (OMX_INDEXTYPE)OMX_ExtraDataVideoEncoderMBInfo) {
                    extraDataParam.nPortIndex = (OMX_U32)PORT_INDEX_OUT;
                }
                DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: extradata: enable for index = %d",
                                  extraDataParam.nIndex);
                err = set_parameter(
                       NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamIndexExtraDataType, &extraDataParam);
                if (err != OMX_ErrorNone) {
                    DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamIndexExtraDataType failed !");
                }
            } while ((token = strtok_r(NULL, "|", &rest)));
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }
    return err;
}
