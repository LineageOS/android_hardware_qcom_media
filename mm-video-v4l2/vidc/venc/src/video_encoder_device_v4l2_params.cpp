/*--------------------------------------------------------------------------
Copyright (c) 2010-2019, The Linux Foundation. All rights reserved.

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

#include "video_encoder_device_v4l2.h"
#include "omx_video_encoder.h"

#undef LOG_TAG
#define LOG_TAG "OMX-VENC: venc_dev"

bool venc_dev::venc_set_param(void *paramData, OMX_INDEXTYPE index)
{
    DEBUG_PRINT_LOW("venc_set_param index 0x%x", index);
    struct v4l2_format fmt;
    struct v4l2_requestbuffers bufreq;
    int ret;
    bool isCBR;

    switch ((int)index) {
        case OMX_IndexParamPortDefinition:
            {
                OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
                portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;

                if (portDefn->nPortIndex == PORT_INDEX_IN) {
                    if (!venc_set_encode_framerate(portDefn->format.video.xFramerate)) {
                        return false;
                    }

                    unsigned long inputformat = venc_get_color_format(portDefn->format.video.eColorFormat);

                    if (m_sVenc_cfg.input_height != portDefn->format.video.nFrameHeight ||
                            m_sVenc_cfg.input_width != portDefn->format.video.nFrameWidth ||
                            m_sInput_buff_property.actualcount != portDefn->nBufferCountActual ||
                            m_sVenc_cfg.inputformat != inputformat) {

                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: port: %u, WxH %lux%lu --> %ux%u, count %lu --> %u, format %#lx --> %#lx",
                            portDefn->nPortIndex, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height,
                            portDefn->format.video.nFrameWidth, portDefn->format.video.nFrameHeight,
                            m_sInput_buff_property.actualcount, portDefn->nBufferCountActual,
                            m_sVenc_cfg.inputformat, inputformat);

                        if (portDefn->nBufferCountActual < m_sInput_buff_property.mincount) {
                            DEBUG_PRINT_LOW("Actual count %u is less than driver mincount %lu on port %u",
                                portDefn->nBufferCountActual, m_sInput_buff_property.mincount, portDefn->nPortIndex);
                            return false;
                        }

                        m_sVenc_cfg.input_height = portDefn->format.video.nFrameHeight;
                        m_sVenc_cfg.input_width = portDefn->format.video.nFrameWidth;
                        m_sVenc_cfg.inputformat = inputformat;
                        m_sInput_buff_property.actualcount = portDefn->nBufferCountActual;

                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
                        fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, pixelformat %#x, colorspace %#x",
                                 fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                                 fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.colorspace);
                            hw_overload = errno == EBUSY;
                            return false;
                        }
                        m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                        bufreq.memory = V4L2_MEMORY_USERPTR;
                        bufreq.count = portDefn->nBufferCountActual;
                        bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                        if (ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("reqbufs failed, type %d, count %d", bufreq.type, bufreq.count);
                            return false;
                        }

                        if (num_input_planes > 1)
                            input_extradata_info.count = m_sInput_buff_property.actualcount + 1;

                        if (!downscalar_enabled) {
                            m_sVenc_cfg.dvs_height = portDefn->format.video.nFrameHeight;
                            m_sVenc_cfg.dvs_width = portDefn->format.video.nFrameWidth;
                        }
                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;

                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("VIDIOC_S_FMT CAPTURE_MPLANE Failed");
                            hw_overload = errno == EBUSY;
                            return false;
                        }

                        m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                    } else {
                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: parameters not changed on port %d",
                            portDefn->nPortIndex);
                    }
                } else if (portDefn->nPortIndex == PORT_INDEX_OUT) {

                    unsigned long codectype = venc_get_codectype(portDefn->format.video.eCompressionFormat);

                    //Don't worry about width/height if downscalar is enabled.
                    if (((m_sVenc_cfg.dvs_height != portDefn->format.video.nFrameHeight ||
                            m_sVenc_cfg.dvs_width != portDefn->format.video.nFrameWidth) && !downscalar_enabled) ||
                            m_sOutput_buff_property.actualcount != portDefn->nBufferCountActual ||
                            m_sVenc_cfg.codectype != codectype) {

                        if (portDefn->nBufferCountActual < m_sOutput_buff_property.mincount) {
                            DEBUG_PRINT_LOW("Actual count %u is less than driver mincount %lu on port %u",
                                portDefn->nBufferCountActual, m_sOutput_buff_property.mincount, portDefn->nPortIndex);
                            return false;
                        }

                        //If downscalar is enabled. Correct width/height is populated no need to replace with port def width/height
                        if (!downscalar_enabled) {
                            DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: port: %u, WxH %lux%lu --> %ux%u, count %lu --> %u, format %#lx --> %#lx",
                                            portDefn->nPortIndex, m_sVenc_cfg.dvs_width, m_sVenc_cfg.dvs_height,
                                            portDefn->format.video.nFrameWidth, portDefn->format.video.nFrameHeight,
                                            m_sOutput_buff_property.actualcount, portDefn->nBufferCountActual,
                                            m_sVenc_cfg.codectype, codectype);
                            m_sVenc_cfg.dvs_height = portDefn->format.video.nFrameHeight;
                            m_sVenc_cfg.dvs_width = portDefn->format.video.nFrameWidth;
                        }

                        m_sVenc_cfg.codectype = codectype;
                        m_sOutput_buff_property.actualcount = portDefn->nBufferCountActual;

                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;
                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, pixelformat %#x",
                                 fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                                 fmt.fmt.pix_mp.pixelformat);
                            hw_overload = errno == EBUSY;
                            return false;
                        }
                        m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                        if (!venc_set_target_bitrate(portDefn->format.video.nBitrate)) {
                            return false;
                        }

                        bufreq.memory = V4L2_MEMORY_USERPTR;
                        bufreq.count = portDefn->nBufferCountActual;
                        bufreq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        if (ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("reqbufs failed, type %d, count %d", bufreq.type, bufreq.count);
                            return false;
                        }

                        if (num_output_planes > 1)
                            output_extradata_info.count = m_sOutput_buff_property.actualcount;
                    } else {
                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: parameters not changed on port %d",
                            portDefn->nPortIndex);
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index (%d) for OMX_IndexParamPortDefinition", portDefn->nPortIndex);
                }
            }
            break;
        case OMX_IndexParamVideoPortFormat:
            {
                OMX_VIDEO_PARAM_PORTFORMATTYPE *portFmt;
                portFmt =(OMX_VIDEO_PARAM_PORTFORMATTYPE *)paramData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoPortFormat");

                if (portFmt->nPortIndex == (OMX_U32) PORT_INDEX_IN) {
                    if (!venc_set_color_format(portFmt->eColorFormat)) {
                        return false;
                    }
                } else if (portFmt->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_encode_framerate(portFmt->xFramerate)) {
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoPortFormat");
                }
                    break;
            }
        case OMX_IndexParamVideoBitrate:
            {
                OMX_VIDEO_PARAM_BITRATETYPE* pParam;
                pParam = (OMX_VIDEO_PARAM_BITRATETYPE*)paramData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoBitrate");

                if (pParam->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_ratectrl_cfg(pParam->eControlRate)) {
                        DEBUG_PRINT_ERROR("ERROR: Rate Control setting failed");
                        return false;
                    }

                    if (!venc_set_target_bitrate(pParam->nTargetBitrate)) {
                        DEBUG_PRINT_ERROR("ERROR: Target Bit Rate setting failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoBitrate");
                }

                break;
            }
        case OMX_IndexParamVideoAvc:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoAvc");
                OMX_VIDEO_PARAM_AVCTYPE* pParam = (OMX_VIDEO_PARAM_AVCTYPE*)paramData;
                OMX_U32 bFrames = 0;

                if (pParam->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    DEBUG_PRINT_LOW("pParam->eProfile :%d ,pParam->eLevel %d",
                            pParam->eProfile,pParam->eLevel);

                    if (!venc_set_profile (pParam->eProfile)) {
                        DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                                pParam->eProfile);
                        return false;
                    }

                    if (set_nP_frames(pParam->nPFrames) == false ||
                        (pParam->nBFrames && set_nB_frames(pParam->nBFrames) == false)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
                        return false;
                    }
                    if (!venc_set_entropy_config (pParam->bEntropyCodingCABAC, pParam->nCabacInitIdc)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting Entropy failed");
                        return false;
                    }
                    if (!venc_set_inloop_filter (pParam->eLoopFilterMode)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting Inloop filter failed");
                        return false;
                    }

                    if (!venc_set_multislice_cfg(V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB, pParam->nSliceHeaderSpacing)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating slice_config");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoAvc");
                }

                //TBD, lot of other variables to be updated, yet to decide
                break;
            }
        case (OMX_INDEXTYPE)OMX_IndexParamVideoVp8:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoVp8");
                OMX_VIDEO_PARAM_VP8TYPE* pParam = (OMX_VIDEO_PARAM_VP8TYPE*)paramData;

                //TODO: Set VP8 level/profile currently based on driver change
                if (!venc_set_profile (pParam->eProfile)) {
                    DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                            pParam->eProfile);
                    return false;
                }
                if(venc_set_vpx_error_resilience(pParam->bErrorResilientMode) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Failed to set vpx error resilience");
                    return false;
                }
                break;
            }
            case (OMX_INDEXTYPE)OMX_IndexParamVideoHevc:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoHevc");
                OMX_VIDEO_PARAM_HEVCTYPE* pParam = (OMX_VIDEO_PARAM_HEVCTYPE*)paramData;

                if (!venc_set_profile (pParam->eProfile)) {
                    DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                                        pParam->eProfile);
                    return false;
                }
                if (!venc_set_inloop_filter(OMX_VIDEO_AVCLoopFilterEnable))
                    DEBUG_PRINT_HIGH("WARN: Request for setting Inloop filter failed for HEVC encoder");

                OMX_U32 fps = m_sVenc_cfg.fps_den ? m_sVenc_cfg.fps_num / m_sVenc_cfg.fps_den : 30;
                OMX_U32 nPFrames = pParam->nKeyFrameInterval > 0 ? pParam->nKeyFrameInterval - 1 : fps - 1;
                if (set_nP_frames(nPFrames) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
                    return false;
                }
                break;
            }
        case (OMX_INDEXTYPE)OMX_IndexParamVideoAndroidImageGrid:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoAndroidImageGrid");

                if (m_codec != OMX_VIDEO_CodingImageHEIC) {
                    DEBUG_PRINT_ERROR("OMX_IndexParamVideoAndroidImageGrid is only set for HEIC (HW tiling)");
                    return true;
                }

                if (!venc_set_grid_enable()) {
                    DEBUG_PRINT_ERROR("ERROR: Failed to set grid-enable");
                    return false;
                }
                break;
            }
        case OMX_IndexParamVideoIntraRefresh:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoIntraRefresh");
                OMX_VIDEO_PARAM_INTRAREFRESHTYPE *intra_refresh_param =
                    (OMX_VIDEO_PARAM_INTRAREFRESHTYPE *)paramData;

                if (intra_refresh_param->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    intra_refresh.irmode     = intra_refresh_param->eRefreshMode;
                    intra_refresh.mbcount    = intra_refresh_param->nCirMBs;
                    intra_refresh.framecount = 0;
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoIntraRefresh");
                }

                break;
            }
        case OMX_IndexParamVideoErrorCorrection:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoErrorCorrection");
                OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *error_resilience =
                    (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *)paramData;

                if (error_resilience->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_error_resilience(error_resilience) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Error Correction failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoErrorCorrection");
                }

                break;
            }
         case OMX_QcomIndexParamVideoSliceSpacing:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_QcomIndexParamVideoSliceSpacing");
                QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE *slice_spacing =
                    (QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE*)paramData;

                if (slice_spacing->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_multislice_cfg(slice_spacing->eSliceMode, slice_spacing->nSliceSize)) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Slice Spacing failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_QcomIndexParamVideoSliceSpacing");
                }

                break;
            }
        case OMX_IndexParamVideoProfileLevelCurrent:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoProfileLevelCurrent");
                OMX_VIDEO_PARAM_PROFILELEVELTYPE *profile_level =
                    (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)paramData;

                if (profile_level->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_profile(profile_level->eProfile)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating Profile");
                        return false;
                    }
                    if (!venc_set_level(profile_level->eLevel)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating level");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoProfileLevelCurrent");
                }

                break;
            }
        case OMX_IndexParamVideoQuantization:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoQuantization");
                OMX_VIDEO_PARAM_QUANTIZATIONTYPE *session_qp =
                    (OMX_VIDEO_PARAM_QUANTIZATIONTYPE *)paramData;
                if (session_qp->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_qp(session_qp->nQpI,
                                session_qp->nQpP,
                                session_qp->nQpB,
                                ENABLE_I_QP | ENABLE_P_QP | ENABLE_B_QP) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Session QP failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoQuantization");
                }

                break;
            }
        case QOMX_IndexParamVideoInitialQp:
            {
                DEBUG_PRINT_LOW("venc_set_param:QOMX_IndexParamVideoInitialQp");
                QOMX_EXTNINDEX_VIDEO_INITIALQP *initial_qp =
                    (QOMX_EXTNINDEX_VIDEO_INITIALQP *)paramData;
                if (initial_qp->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_qp(initial_qp->nQpI,
                                initial_qp->nQpP,
                                initial_qp->nQpB,
                                initial_qp->bEnableInitQp) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Initial QP failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for QOMX_IndexParamVideoInitialQp");
                }

                break;
            }
        case OMX_QcomIndexParamVideoIPBQPRange:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_QcomIndexParamVideoIPBQPRange");
                OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE *session_qp_range =
                    (OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE *)paramData;
                if(session_qp_range->nPortIndex == (OMX_U32)PORT_INDEX_OUT) {
                    if ( venc_set_session_qp_range (session_qp_range) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting QP range failed");
                        return false;
                    }
                }

                break;
            }
        case OMX_ExtraDataVideoLTRInfo:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_ExtraDataVideoLTRInfo");
                OMX_BOOL extra_data =  *(OMX_BOOL *)(paramData);
                if (venc_set_extradata(OMX_ExtraDataVideoLTRInfo, extra_data) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_ExtraDataVideoLTRInfo failed");
                    return false;
                }
                extradata = true;
                break;
            }
        case OMX_QcomIndexParamSequenceHeaderWithIDR:
            {
                PrependSPSPPSToIDRFramesParams * pParam =
                    (PrependSPSPPSToIDRFramesParams *)paramData;

                DEBUG_PRINT_LOW("set inband sps/pps: %d", pParam->bEnable);
                if(venc_set_inband_video_header(pParam->bEnable) == false) {
                    DEBUG_PRINT_ERROR("ERROR: set inband sps/pps failed");
                    return false;
                }

                break;
            }
        case OMX_QcomIndexParamAUDelimiter:
            {
                OMX_QCOM_VIDEO_CONFIG_AUD * pParam =
                    (OMX_QCOM_VIDEO_CONFIG_AUD *)paramData;

                DEBUG_PRINT_LOW("set AU Delimiters: %d", pParam->bEnable);
                if(venc_set_au_delimiter(pParam->bEnable) == false) {
                    DEBUG_PRINT_ERROR("ERROR: set AU delimiter failed");
                    return false;
                }

                break;
            }
        case OMX_QcomIndexConfigH264EntropyCodingCabac:
            {
                QOMX_VIDEO_H264ENTROPYCODINGTYPE * pParam =
                    (QOMX_VIDEO_H264ENTROPYCODINGTYPE *)paramData;

                DEBUG_PRINT_LOW("set Entropy info : %d", pParam->bCabac);
                if(venc_set_entropy_config (pParam->bCabac, 0) == false) {
                    DEBUG_PRINT_ERROR("ERROR: set Entropy failed");
                    return false;
                }

                break;
            }
        case OMX_QcomIndexParamH264VUITimingInfo:
            {
                OMX_QCOM_VIDEO_PARAM_VUI_TIMING_INFO *pParam =
                        (OMX_QCOM_VIDEO_PARAM_VUI_TIMING_INFO *)paramData;
                DEBUG_PRINT_LOW("Set VUI timing info: %d", pParam->bEnable);
                if(venc_set_vui_timing_info(pParam->bEnable) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Failed to set vui timing info to %d", pParam->bEnable);
                    return false;
                } else {
                    vui_timing_info.enabled = (unsigned int) pParam->bEnable;
                }
                break;
            }
        case OMX_QcomIndexParamVideoLTRCount:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_QcomIndexParamVideoLTRCount");
                OMX_QCOM_VIDEO_PARAM_LTRCOUNT_TYPE* pParam =
                        (OMX_QCOM_VIDEO_PARAM_LTRCOUNT_TYPE*)paramData;
                if (venc_set_ltrcount(pParam->nCount) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Enable LTR mode failed");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexParamBatchSize:
            {
                OMX_PARAM_U32TYPE* pParam =
                    (OMX_PARAM_U32TYPE*)paramData;

                if (pParam->nPortIndex == PORT_INDEX_OUT) {
                    DEBUG_PRINT_ERROR("For the moment, client-driven batching not supported"
                            " on output port");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexParamVencAspectRatio:
            {
                if (!venc_set_aspectratio(paramData)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QcomIndexParamVencAspectRatio failed");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexParamVideoEnableRoiInfo:
            {
                struct v4l2_control control;
                OMX_BOOL extra_data =  *(OMX_BOOL *)(paramData);
                if (extra_data == OMX_FALSE) {
                    DEBUG_PRINT_INFO("OMX_QTIIndexParamVideoEnableRoiInfo: bEnableRoiInfo is false");
                    break;
                }
                if (m_sVenc_cfg.codectype != V4L2_PIX_FMT_H264 &&
                        m_sVenc_cfg.codectype != V4L2_PIX_FMT_HEVC) {
                    DEBUG_PRINT_ERROR("OMX_QTIIndexParamVideoEnableRoiInfo is not supported for %lu codec", m_sVenc_cfg.codectype);
                    return false;
                }
                control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
                control.value = EXTRADATA_ENC_INPUT_ROI;
                DEBUG_PRINT_LOW("Setting param OMX_QTIIndexParamVideoEnableRoiInfo");
                if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QTIIndexParamVideoEnableRoiInfo failed");
                    return false;
                }
                m_roi_enabled = true;
                break;
            }
        case OMX_QTIIndexParamLowLatencyMode:
            {
                QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE *pParam =
                    (QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE*)paramData;

                if (!venc_set_lowlatency_mode(pParam->bEnableLowLatencyMode)) {
                     DEBUG_PRINT_ERROR("Setting low latency mode failed");
                     return false;
                }
                break;
            }
        case OMX_IndexParamAndroidVideoTemporalLayering:
            {
                OMX_VIDEO_PARAM_ANDROID_TEMPORALLAYERINGTYPE hierData;
                memcpy(&hierData, paramData, sizeof(hierData));

                // Update temporal_layers_config with input param
                if (hierData.nPLayerCountActual < OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS) {
                    temporal_layers_config.nPLayers = hierData.nPLayerCountActual;
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed");
                    return false;
                }
                temporal_layers_config.ePattern = hierData.ePattern;
                temporal_layers_config.hier_mode = HIER_P;
                temporal_layers_config.nMaxLayers = hierData.nLayerCountMax;
                temporal_layers_config.nMaxBLayers = hierData.nBLayerCountMax;
                temporal_layers_config.nBLayers = hierData.nBLayerCountActual;
                // Resetting to zero as we are sending all bitrate ratios to kernel
                memset(&temporal_layers_config.nTemporalLayerBitrateRatio, 0x0, sizeof(OMX_U32)*OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS);
                for (OMX_U32 i = 0; i < temporal_layers_config.nPLayers; ++i) {
                    temporal_layers_config.nTemporalLayerBitrateRatio[i] = hierData.nBitrateRatios[i];
                }

                if (OMX_ErrorNone != venc_set_max_hierp_layer()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting max hierp layers");
                    return false;
                }
                if (OMX_ErrorNone != venc_set_hierp_layer()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting hierp layers");
                    return false;
                }
                if (OMX_ErrorNone != venc_set_bitrate_ratios()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting bitrate ratios");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                mUseAVTimerTimestamps = pParam->bEnable == OMX_TRUE;
                DEBUG_PRINT_INFO("AVTimer timestamps enabled");
                break;
            }
        case OMX_QcomIndexParamVideoDownScalar:
            {
                QOMX_INDEXDOWNSCALAR *pParam = (QOMX_INDEXDOWNSCALAR *)paramData;
                downscalar_enabled = pParam->bEnable;

                DEBUG_PRINT_INFO("Downscalar settings: Enabled : %d Width : %u Height %u",
                    pParam->bEnable, pParam->nOutputWidth, pParam->nOutputHeight);
                if (downscalar_enabled) {
                    m_sVenc_cfg.dvs_width = pParam->nOutputWidth;
                    m_sVenc_cfg.dvs_height = pParam->nOutputHeight;

                    memset(&fmt, 0, sizeof(fmt));
                    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                    fmt.fmt.pix_mp.width = pParam->nOutputWidth;
                    fmt.fmt.pix_mp.height = pParam->nOutputHeight;
                    fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;
                    if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                        DEBUG_PRINT_ERROR("Failed to set format on capture port");
                        return false;
                    }
                    m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
                }
                break;
            }
        case OMX_QTIIndexParamColorSpaceConversion:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                csc_enable = pParam->bEnable;
                DEBUG_PRINT_INFO("CSC settings: Enabled : %d ", pParam->bEnable);
                break;
            }
        case OMX_QTIIndexParamEnableLinearColorFormat:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                mUseLinearColorFormat = pParam->bEnable ? REQUEST_LINEAR_COLOR_ALL : 0;
                DEBUG_PRINT_INFO("Linear Color Format Enabled : %d ", pParam->bEnable);
                break;
            }
        case OMX_QTIIndexParamVideoEnableBlur:
            {
                OMX_QTI_VIDEO_CONFIG_BLURINFO *pParam = (OMX_QTI_VIDEO_CONFIG_BLURINFO *)paramData;
                if (!venc_set_blur_resolution(pParam)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QTIIndexParamVideoEnableBlur failed");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexParamNativeRecorder:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                if (!set_native_recoder(pParam->bEnable)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QTIIndexParamNativeRecorder failed");
                    return false;
                }
                DEBUG_PRINT_INFO("Native recorder encode session %d", pParam->bEnable);
                break;
            }
        default:
            DEBUG_PRINT_ERROR("ERROR: Unsupported parameter in venc_set_param: %u",
                    index);
            break;
    }

    return true;
}

bool venc_dev::venc_set_inband_video_header(OMX_BOOL enable)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR;
    control.value = V4L2_MPEG_MSM_VIDC_DISABLE;
    if(enable) {
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    }

    DEBUG_PRINT_HIGH("Set inband sps/pps: %d", enable);
    if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control) < 0) {
        DEBUG_PRINT_ERROR("Request for inband sps/pps failed");
        return false;
    }
    return true;
}

bool venc_dev::venc_set_au_delimiter(OMX_BOOL enable)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_AU_DELIMITER;
    if(enable) {
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    } else {
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;
    }

    DEBUG_PRINT_HIGH("Set AU delimiters: %d", enable);
    if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control) < 0) {
        DEBUG_PRINT_ERROR("Request for AU delimiters failed");
        return false;
    }
    return true;
}

bool venc_dev::venc_set_extradata(OMX_U32 extra_data, OMX_BOOL enable)
{
    struct v4l2_control control;

    DEBUG_PRINT_HIGH("venc_set_extradata:: %x", (int) extra_data);

    if (enable == OMX_FALSE) {
        /* No easy way to turn off extradata to the driver
         * at the moment */
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
    switch (extra_data) {
        case OMX_ExtraDataVideoLTRInfo:
            control.value = EXTRADATA_ADVANCED;
            break;
        default:
            DEBUG_PRINT_ERROR("Unrecognized extradata index 0x%x", (unsigned int)extra_data);
            return false;
    }

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("ERROR: Request for setting extradata (%x) failed %d",
                (unsigned int)extra_data, errno);
        return false;
    }

    return true;
}

bool venc_dev::venc_set_session_qp_range(OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE* qp_range)
{
    int rc;
    struct v4l2_control control[2];

    control[0].id = V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP;
    control[0].value = qp_range->minIQP | (qp_range->minPQP << 8) | (qp_range->minBQP << 16);

    control[1].id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP;
    control[1].value = qp_range->maxIQP | (qp_range->maxPQP << 8) | (qp_range->maxBQP << 16);

    for(int i=0; i<2; i++) {
        if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control[i])) {
            DEBUG_PRINT_ERROR("Failed to set QP %s range", i==0?"MIN":"MAX");
            return false;
        }
    }

    session_ipb_qp_values.min_qp_packed = control[0].value;
    session_ipb_qp_values.max_qp_packed = control[1].value;

    return true;
}

bool venc_dev::venc_set_profile(OMX_U32 eProfile)
{
    int rc;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_profile:: eProfile = %u",
            (unsigned int)eProfile);

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        control.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        //In driver VP8 profile is hardcoded. No need to set anything from here
        return true;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        control.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
    } else {
        DEBUG_PRINT_ERROR("Wrong CODEC");
        return false;
    }

    if (!profile_level_converter::convert_omx_profile_to_v4l2(m_sVenc_cfg.codectype, eProfile, &control.value)) {
        DEBUG_PRINT_ERROR("Cannot find v4l2 profile for OMX profile : %d Codec : %lu ",
                          eProfile, m_sVenc_cfg.codectype);
        return false;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    codec_profile.profile = control.value;
    return true;
}

bool venc_dev::venc_set_level(OMX_U32 eLevel)
{
    int rc;
    struct v4l2_control control;
    unsigned int tier = V4L2_MPEG_VIDEO_HEVC_TIER_HIGH;

    DEBUG_PRINT_LOW("venc_set_level:: eLevel = %u",
                    (unsigned int)eLevel);

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        control.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
        control.value = V4L2_MPEG_VIDEO_H264_LEVEL_UNKNOWN;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL;
        control.value = V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        control.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
        control.value = V4L2_MPEG_VIDEO_HEVC_LEVEL_UNKNOWN;
        profile_level.tier = tier;
    }
    else {
        DEBUG_PRINT_ERROR("Wrong CODEC");
        return false;
    }

    /* Set default level */
    profile_level.level = control.value;
    if (eLevel != OMX_VIDEO_LEVEL_UNKNOWN) {
        if (!profile_level_converter::convert_omx_level_to_v4l2(m_sVenc_cfg.codectype, eLevel, &control.value, &tier)) {
            DEBUG_PRINT_LOW("Warning: Cannot find v4l2 level for OMX level : %d" \
                            " Codec : %lu Setting unknown level",
                              eLevel, m_sVenc_cfg.codectype);
        } else {
            DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d",
                                                    control.id, control.value);
            rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
            if (rc) {
                DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d",
                                                    control.id, control.value);
                return false;
            }
            DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d",
                                                    control.id, control.value);

            if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
                struct v4l2_control control_tier = {
                    .id = V4L2_CID_MPEG_VIDEO_HEVC_TIER,
                    .value = (signed int)tier
                };
                DEBUG_PRINT_ERROR("Calling IOCTL set tier control for HEVC, id %#x, value %d",
                                  control_tier.id, control_tier.value);

                rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control_tier);
                if (rc) {
                    DEBUG_PRINT_ERROR("Failed to set tier control for HEVC, id %#x, value %d",
                                      control_tier.id, control_tier.value);
                } else {
                    profile_level.tier = control_tier.value;
                }
            }
            profile_level.level = control.value;
        }
    }

    return true;
}

bool venc_dev::venc_set_grid_enable()
{
    int rc;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_grid_enable");
    control.id = V4L2_CID_MPEG_VIDC_IMG_GRID_SIZE;
    control.value = 1; // TODO: DO we need to get this value from input argument?
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    mIsGridset = true;
    return true;
}

bool venc_dev::venc_set_entropy_config(OMX_BOOL enable, OMX_U32 i_cabac_level)
{
    int rc = 0;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_entropy_config: CABAC = %u level: %u", enable, (unsigned int)i_cabac_level);

    if (enable && (codec_profile.profile != V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) &&
            (codec_profile.profile != V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)) {
        control.value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC;
    } else if (!enable) {
        control.value =  V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC;
    } else {
        DEBUG_PRINT_ERROR("Invalid Entropy mode for Baseline Profile");
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    entropy.longentropysel = control.value;

    return true;
}

bool venc_dev::venc_set_multislice_cfg(OMX_U32 nSlicemode, OMX_U32 nSlicesize)
{
    int rc;
    int slice_id = 0;
    struct v4l2_control control;
    bool status = true;

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H263 || nSlicesize == 0) {
        nSlicemode = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
        nSlicesize = 0;
    }

    if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB) {
        if (!venc_validate_range(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, nSlicesize)) {
            DEBUG_PRINT_ERROR("Invalid settings, hardware doesn't support %u as slicesize", nSlicesize);
            return false;
        }
        slice_id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB;

    } else if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES) {
        if (!venc_validate_range(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES, nSlicesize)) {
            DEBUG_PRINT_ERROR("Invalid settings, hardware doesn't support %u as slicesize", nSlicesize);
            return false;
        }
        slice_id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES;

    } else if (nSlicesize) {
        DEBUG_PRINT_ERROR("Invalid settings, unexpected slicemode = %u and slice size = %u", nSlicemode, nSlicesize);
        return false;
    }

    control.id    = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE;
    control.value = nSlicemode;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE) {
        return status;
    }

    control.id    = slice_id;
    control.value = nSlicesize;

    DEBUG_PRINT_LOW("Calling SLICE_MB IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    multislice.mslice_mode = nSlicemode;
    multislice.mslice_size = nSlicesize;

    return status;
}

bool venc_dev::venc_set_error_resilience(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* error_resilience)
{
    bool status = true;
    struct venc_headerextension hec_cfg;
    struct venc_multiclicecfg multislice_cfg;
    int rc;
    OMX_U32 resynchMarkerSpacingBytes = 0;
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_MPEG4) {
        if (error_resilience->bEnableHEC) {
            hec_cfg.header_extension = 1;
        } else {
            hec_cfg.header_extension = 0;
        }

        hec.header_extension = error_resilience->bEnableHEC;
    }

    if (error_resilience->bEnableRVLC) {
        DEBUG_PRINT_ERROR("RVLC is not Supported");
        return false;
    }

    if (( m_sVenc_cfg.codectype != V4L2_PIX_FMT_H263) &&
            (error_resilience->bEnableDataPartitioning)) {
        DEBUG_PRINT_ERROR("DataPartioning are not Supported for MPEG4/H264");
        return false;
    }

    if (error_resilience->nResynchMarkerSpacing) {
        resynchMarkerSpacingBytes = error_resilience->nResynchMarkerSpacing;
        resynchMarkerSpacingBytes = ALIGN(resynchMarkerSpacingBytes, 8) >> 3;
    }

    status = venc_set_multislice_cfg(V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES, resynchMarkerSpacingBytes);

    return status;
}

bool venc_dev::venc_set_inloop_filter(OMX_VIDEO_AVCLOOPFILTERTYPE loopfilter)
{
    int rc;
    struct v4l2_control control;
    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE;
    control.value=0;

    if (loopfilter == OMX_VIDEO_AVCLoopFilterEnable) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED;
    } else if (loopfilter == OMX_VIDEO_AVCLoopFilterDisable) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED;
    } else if (loopfilter == OMX_VIDEO_AVCLoopFilterDisableSliceBoundary) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    dbkfilter.db_mode=control.value;

    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA;
    control.value=0;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA;
    control.value=0;
    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);


    dbkfilter.slicealpha_offset = dbkfilter.slicebeta_offset = 0;
    return true;
}

unsigned long venc_dev::venc_get_codectype(OMX_VIDEO_CODINGTYPE eCompressionFormat)
{
    unsigned long codectype = V4L2_PIX_FMT_H264;

    switch ((int)eCompressionFormat) {
    case OMX_VIDEO_CodingAVC:
        codectype = V4L2_PIX_FMT_H264;
        break;
    case OMX_VIDEO_CodingVP8:
        codectype = V4L2_PIX_FMT_VP8;
        break;
    case OMX_VIDEO_CodingVP9:
        codectype = V4L2_PIX_FMT_VP9;
        break;
    case OMX_VIDEO_CodingHEVC:
    case OMX_VIDEO_CodingImageHEIC:
        codectype = V4L2_PIX_FMT_HEVC;
        break;
    default:
        DEBUG_PRINT_ERROR("Unsupported eCompressionFormat %#x", eCompressionFormat);
        codectype = V4L2_PIX_FMT_H264;
        break;
    }

    return codectype;
}

bool venc_dev::venc_set_ratectrl_cfg(OMX_VIDEO_CONTROLRATETYPE eControlRate)
{
    bool status = true;
    struct v4l2_control control;
    int rc = 0;

    control.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
    control.value = !!((OMX_U32)eControlRate ^ (OMX_U32)OMX_Video_ControlRateDisable);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set RC_ENABLE");
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
    int temp = eControlRate;
    switch ((OMX_U32)eControlRate) {
        case OMX_Video_ControlRateDisable:
            control.value = -1;
            break;
        case OMX_Video_ControlRateVariableSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
            break;
        case OMX_Video_ControlRateVariable:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
            break;
        case OMX_Video_ControlRateConstantSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR_VFR;
            break;
        case OMX_Video_ControlRateConstant:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
            break;
        case QOMX_Video_ControlRateMaxBitrate:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_MBR;
            break;
        case QOMX_Video_ControlRateMaxBitrateSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_MBR_VFR;
            break;
        case OMX_Video_ControlRateConstantQuality:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CQ;
            break;
        default:
            status = false;
            break;
    }

    if (status && control.value != -1) {

        DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set control");
            return false;
        }

        DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

        rate_ctrl.rcmode = control.value;
    }

    return status;
}

bool venc_dev::venc_set_aspectratio(void *nSar)
{
    int rc;
    struct v4l2_control control;
    struct v4l2_ext_control ctrl[2];
    struct v4l2_ext_controls controls;
    QOMX_EXTNINDEX_VIDEO_VENC_SAR *sar;

    sar = (QOMX_EXTNINDEX_VIDEO_VENC_SAR *) nSar;

    ctrl[0].id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH;
    ctrl[0].value = sar->nSARWidth;
    ctrl[1].id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT;
    ctrl[1].value = sar->nSARHeight;

    controls.count = 2;
    controls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
    controls.controls = ctrl;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x val=%d, id=%x val=%d",
                    controls.controls[0].id, controls.controls[0].value,
                    controls.controls[1].id, controls.controls[1].value);

    rc = ioctl(m_nDriver_fd, VIDIOC_S_EXT_CTRLS, &controls);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set SAR %d", rc);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x val=%d, id=%x val=%d",
                    controls.controls[0].id, controls.controls[0].value,
                    controls.controls[1].id, controls.controls[1].value);
    return true;
}

bool venc_dev::venc_set_lowlatency_mode(OMX_BOOL enable)
{
    int rc = 0;
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_LOWLATENCY_MODE;
    if (enable)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set lowlatency control");
        return false;
    }
    low_latency_mode = enable;
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_vui_timing_info(OMX_BOOL enable)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_VUI_TIMING_INFO;

    if (enable)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set VUI timing info control");
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, value=%d", control.id, control.value);
    return true;
}

bool venc_dev::venc_set_peak_bitrate(OMX_U32 nPeakBitrate)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_PEAK;
    control.value = nPeakBitrate;

    DEBUG_PRINT_LOW("venc_set_peak_bitrate: bitrate = %u", (unsigned int)nPeakBitrate);

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set peak bitrate control");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_vpx_error_resilience(OMX_BOOL enable)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE;

    if (enable)
        control.value = 1;
    else
        control.value = 0;

    DEBUG_PRINT_LOW("venc_set_vpx_error_resilience: %d", control.value);

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);

    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set VPX Error Resilience");
        return false;
    }
    vpx_err_resilience.enable = 1;
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    return true;
}


