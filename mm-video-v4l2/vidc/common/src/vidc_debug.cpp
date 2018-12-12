/*--------------------------------------------------------------------------
Copyright (c) 2010-2018, The Linux Foundation. All rights reserved.

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

#include "vidc_debug.h"

void print_omx_buffer(const char *str, OMX_BUFFERHEADERTYPE *pHeader)
{
    if (!pHeader)
        return;

    DEBUG_PRINT_HIGH("%s: Header %p buffer %p alloclen %d offset %d filledlen %d timestamp %lld flags %#x",
        str, pHeader, pHeader->pBuffer, pHeader->nAllocLen,
        pHeader->nOffset, pHeader->nFilledLen,
        pHeader->nTimeStamp, pHeader->nFlags);
}

void print_v4l2_buffer(const char *str, struct v4l2_buffer *v4l2)
{
    if (!v4l2)
        return;

    if (v4l2->length == 1)
        DEBUG_PRINT_HIGH(
            "%s: %s: idx %2d userptr %#lx fd %d off %d size %d filled %d flags %#x\n",
            str, v4l2->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
            "OUTPUT" : "CAPTURE", v4l2->index,
            v4l2->m.planes[0].m.userptr, v4l2->m.planes[0].reserved[0],
            v4l2->m.planes[0].reserved[1], v4l2->m.planes[0].length,
            v4l2->m.planes[0].bytesused, v4l2->flags);
    else
        DEBUG_PRINT_HIGH(
            "%s: %s: idx %2d userptr %#lx fd %d off %d size %d filled %d flags %#x, extradata: fd %d off %d size %d filled %d\n",
            str, v4l2->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
            "OUTPUT" : "CAPTURE", v4l2->index,
            v4l2->m.planes[0].m.userptr, v4l2->m.planes[0].reserved[0],
            v4l2->m.planes[0].reserved[1], v4l2->m.planes[0].length,
            v4l2->m.planes[0].bytesused, v4l2->m.planes[1].reserved[0],
            v4l2->flags, v4l2->m.planes[1].reserved[1],
            v4l2->m.planes[1].length, v4l2->m.planes[1].bytesused);
}

void print_debug_color_aspects(ColorAspects *a, const char *prefix)
{
  DEBUG_PRINT_HIGH("%s : Color aspects : Primaries = %d(%s) Range = %d(%s) Tx = %d(%s) Matrix = %d(%s)",
                   prefix, a->mPrimaries, asString(a->mPrimaries), a->mRange, asString(a->mRange),
                   a->mTransfer, asString(a->mTransfer), a->mMatrixCoeffs, asString(a->mMatrixCoeffs));

}

void print_debug_hdr_color_info(HDRStaticInfo *hdr_info, const char *prefix)
{
    if (!hdr_info->mID) {
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mR.x = %d mR.y = %d", prefix,
                         hdr_info->sType1.mR.x, hdr_info->sType1.mR.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mG.x = %d mG.y = %d", prefix,
                         hdr_info->sType1.mG.x, hdr_info->sType1.mG.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mB.x = %d mB.y = %d", prefix,
                         hdr_info->sType1.mB.x, hdr_info->sType1.mB.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mW.x = %d mW.y = %d", prefix,
                         hdr_info->sType1.mW.x, hdr_info->sType1.mW.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: maxDispLum = %d minDispLum = %d", prefix,
                         hdr_info->sType1.mMaxDisplayLuminance, hdr_info->sType1.mMinDisplayLuminance);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo CLL: CLL = %d FLL = %d", prefix,
                        hdr_info->sType1.mMaxContentLightLevel, hdr_info->sType1.mMaxFrameAverageLightLevel);
    }

}

void print_debug_hdr_color_info_mdata(ColorMetaData* color_mdata)
{
    DEBUG_PRINT_LOW("setMetaData COLOR_METADATA : color_primaries = %u, range = %u, transfer = %u, matrix = %u",
                    color_mdata->colorPrimaries, color_mdata->range,
                    color_mdata->transfer, color_mdata->matrixCoefficients);

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : rgbPrimaries[%d][%d] = %d", i, j, color_mdata->masteringDisplayInfo.primaries.rgbPrimaries[i][j]);
        }
    }

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : whitepoint[0] = %d whitepoint[1] = %d",
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[0],
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[1]);

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : maxDispLum = %d minDispLum = %d",
                    color_mdata->masteringDisplayInfo.maxDisplayLuminance,
                    color_mdata->masteringDisplayInfo.minDisplayLuminance);

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : maxCLL = %d maxFLL = %d",
                    color_mdata->contentLightLevel.maxContentLightLevel,
                    color_mdata->contentLightLevel.minPicAverageLightLevel);
}

void print_debug_hdr10plus_metadata(ColorMetaData& color_mdata)
{
    DEBUG_PRINT_LOW("HDR10+ valid data length: %d", color_mdata.dynamicMetaDataLen);
    for (uint32_t i = 0 ; i < color_mdata.dynamicMetaDataLen && i+3 < HDR_DYNAMIC_META_DATA_SZ; i=i+4) {
        DEBUG_PRINT_LOW("HDR10+ mdata: %02X %02X %02X %02X", color_mdata.dynamicMetaDataPayload[i],
                        color_mdata.dynamicMetaDataPayload[i+1],
                        color_mdata.dynamicMetaDataPayload[i+2],
                        color_mdata.dynamicMetaDataPayload[i+3]);
    }

}

void print_debug_extradata(OMX_OTHER_EXTRADATATYPE *extra)
{
    if (!extra)
        return;


    DEBUG_PRINT_HIGH(
            "============== Extra Data ==============\n"
            "           Size: %u\n"
            "        Version: %u\n"
            "      PortIndex: %u\n"
            "           Type: %x\n"
            "       DataSize: %u",
            (unsigned int)extra->nSize, (unsigned int)extra->nVersion.nVersion,
            (unsigned int)extra->nPortIndex, extra->eType, (unsigned int)extra->nDataSize);

    if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataInterlaceFormat) {
        OMX_STREAMINTERLACEFORMAT *intfmt = (OMX_STREAMINTERLACEFORMAT *)(void *)extra->data;
        DEBUG_PRINT_HIGH(
                "------ Interlace Format ------\n"
                "                Size: %u\n"
                "             Version: %u\n"
                "           PortIndex: %u\n"
                " Is Interlace Format: %d\n"
                "   Interlace Formats: %u\n"
                "=========== End of Interlace ===========",
                (unsigned int)intfmt->nSize, (unsigned int)intfmt->nVersion.nVersion, (unsigned int)intfmt->nPortIndex,
                intfmt->bInterlaceFormat, (unsigned int)intfmt->nInterlaceFormats);
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataFrameInfo) {
        OMX_QCOM_EXTRADATA_FRAMEINFO *fminfo = (OMX_QCOM_EXTRADATA_FRAMEINFO *)(void *)extra->data;

        DEBUG_PRINT_HIGH(
                "-------- Frame Format --------\n"
                "             Picture Type: %d\n"
                "           Interlace Type: %d\n"
                " Pan Scan Total Frame Num: %u\n"
                "   Concealed Macro Blocks: %u\n"
                "        Recovery SEI Flag: %u\n"
                "               frame rate: %u\n"
                "               Time Stamp: %llu\n"
                "           Aspect Ratio X: %u\n"
                "           Aspect Ratio Y: %u",
                fminfo->ePicType,
                fminfo->interlaceType,
                (unsigned int)fminfo->panScan.numWindows,
                (unsigned int)fminfo->nConcealedMacroblocks,
                (unsigned int)fminfo->nRecoverySeiFlag,
                (unsigned int)fminfo->nFrameRate,
                fminfo->nTimeStamp,
                (unsigned int)fminfo->aspectRatio.aspectRatioX,
                (unsigned int)fminfo->aspectRatio.aspectRatioY);

        for (OMX_U32 i = 0; i < fminfo->panScan.numWindows; i++) {
            DEBUG_PRINT_HIGH(
                    "------------------------------"
                    "     Pan Scan Frame Num: %u\n"
                    "            Rectangle x: %d\n"
                    "            Rectangle y: %d\n"
                    "           Rectangle dx: %d\n"
                    "           Rectangle dy: %d",
                    (unsigned int)i, (unsigned int)fminfo->panScan.window[i].x, (unsigned int)fminfo->panScan.window[i].y,
                    (unsigned int)fminfo->panScan.window[i].dx, (unsigned int)fminfo->panScan.window[i].dy);
        }

        DEBUG_PRINT_HIGH("========= End of Frame Format ==========");
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataFramePackingArrangement) {
        OMX_QCOM_FRAME_PACK_ARRANGEMENT *framepack = (OMX_QCOM_FRAME_PACK_ARRANGEMENT *)(void *)extra->data;
        DEBUG_PRINT_HIGH(
                "------------------ Framepack Format ----------\n"
                "                           id: %u \n"
                "                  cancel_flag: %u \n"
                "                         type: %u \n"
                " quincunx_sampling_flagFormat: %u \n"
                "  content_interpretation_type: %u \n"
                "        spatial_flipping_flag: %u \n"
                "          frame0_flipped_flag: %u \n"
                "             field_views_flag: %u \n"
                " current_frame_is_frame0_flag: %u \n"
                "   frame0_self_contained_flag: %u \n"
                "   frame1_self_contained_flag: %u \n"
                "       frame0_grid_position_x: %u \n"
                "       frame0_grid_position_y: %u \n"
                "       frame1_grid_position_x: %u \n"
                "       frame1_grid_position_y: %u \n"
                "                reserved_byte: %u \n"
                "            repetition_period: %u \n"
                "               extension_flag: %u \n"
                "================== End of Framepack ===========",
                (unsigned int)framepack->id,
                (unsigned int)framepack->cancel_flag,
                (unsigned int)framepack->type,
                (unsigned int)framepack->quincunx_sampling_flag,
                (unsigned int)framepack->content_interpretation_type,
                (unsigned int)framepack->spatial_flipping_flag,
                (unsigned int)framepack->frame0_flipped_flag,
                (unsigned int)framepack->field_views_flag,
                (unsigned int)framepack->current_frame_is_frame0_flag,
                (unsigned int)framepack->frame0_self_contained_flag,
                (unsigned int)framepack->frame1_self_contained_flag,
                (unsigned int)framepack->frame0_grid_position_x,
                (unsigned int)framepack->frame0_grid_position_y,
                (unsigned int)framepack->frame1_grid_position_x,
                (unsigned int)framepack->frame1_grid_position_y,
                (unsigned int)framepack->reserved_byte,
                (unsigned int)framepack->repetition_period,
                (unsigned int)framepack->extension_flag);
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataQP) {
        OMX_QCOM_EXTRADATA_QP * qp = (OMX_QCOM_EXTRADATA_QP *)(void *)extra->data;
        DEBUG_PRINT_HIGH(
                "---- QP (Frame quantization parameter) ----\n"
                "              Frame QP: %u \n"
                "       Sum of Frame QP: %u \n"
                "     Sum of Skipped QP: %u \n"
                "    Num Skipped Blocks: %u \n"
                "          Total Blocks: %u \n"
                "================ End of QP ================\n",
                (unsigned int)qp->nQP,(unsigned int)qp->nQPSum,
                (unsigned int)qp->nSkipQPSum,(unsigned int)qp->nSkipNumBlocks,
                (unsigned int)qp->nTotalNumBlocks);
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataInputBitsInfo) {
        OMX_QCOM_EXTRADATA_BITS_INFO * bits = (OMX_QCOM_EXTRADATA_BITS_INFO *)(void *)extra->data;
        DEBUG_PRINT_HIGH(
                "--------- Input bits information --------\n"
                "    Header bits: %u \n"
                "     Frame bits: %u \n"
                "===== End of Input bits information =====\n",
                (unsigned int)bits->header_bits, (unsigned int)bits->frame_bits);
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataMP2UserData) {
        OMX_QCOM_EXTRADATA_USERDATA *userdata = (OMX_QCOM_EXTRADATA_USERDATA *)(void *)extra->data;
        OMX_U8 *data_ptr = (OMX_U8 *)userdata->data;
        OMX_U32 userdata_size = extra->nDataSize - sizeof(userdata->type);
        OMX_U32 i = 0;
        DEBUG_PRINT_HIGH(
                "--------------  Userdata  -------------\n"
                "    Stream userdata type: %u\n"
                "          userdata size: %u\n"
                "    STREAM_USERDATA:",
                (unsigned int)userdata->type, (unsigned int)userdata_size);
                for (i = 0; i < userdata_size; i+=4) {
                    DEBUG_PRINT_HIGH("        %x %x %x %x",
                        data_ptr[i], data_ptr[i+1],
                        data_ptr[i+2], data_ptr[i+3]);
                }
        DEBUG_PRINT_HIGH(
                "=========== End of Userdata ===========");
    } else if (extra->eType == (OMX_EXTRADATATYPE)OMX_ExtraDataOutputCropInfo) {
        OMX_QCOM_OUTPUT_CROP *outputcrop_info = (OMX_QCOM_OUTPUT_CROP*)(void *)extra->data;
        DEBUG_PRINT_HIGH(
            "------------------ output crop ----------\n"
            "                         left: %u \n"
            "                          top: %u \n"
            "                display_width: %u \n"
            "               display_height: %u \n"
            "                        width: %u \n"
            "                       height: %u \n"
            "                    frame_num: %u \n"
            "                  bit_depth_y: %u \n"
            "                  bit_depth_c: %u \n",
            (unsigned int)outputcrop_info->left,
            (unsigned int)outputcrop_info->top,
            (unsigned int)outputcrop_info->display_width,
            (unsigned int)outputcrop_info->display_height,
            (unsigned int)outputcrop_info->width,
            (unsigned int)outputcrop_info->height,
            (unsigned int)outputcrop_info->frame_num,
            (unsigned int)outputcrop_info->bit_depth_y,
            (unsigned int)outputcrop_info->bit_depth_c);
        for(unsigned int m=0; m<outputcrop_info->misr_info[0].misr_set; m++) {
            DEBUG_PRINT_HIGH(
            "     top field: misr_dpb_luma(%d): %u \n"
            "   top field: misr_dpb_chroma(%d): %u \n"
            "     top field: misr_opb_luma(%d): %u \n"
            "   top field: misr_opb_chroma(%d): %u \n",
            m, (unsigned int)outputcrop_info->misr_info[0].misr_dpb_luma[m],
            m, (unsigned int)outputcrop_info->misr_info[0].misr_dpb_chroma[m],
            m, (unsigned int)outputcrop_info->misr_info[0].misr_opb_luma[m],
            m, (unsigned int)outputcrop_info->misr_info[0].misr_opb_chroma[m]);
        }
        for(unsigned int m=0; m<outputcrop_info->misr_info[1].misr_set; m++) {
            DEBUG_PRINT_HIGH(
            "  bottom field: misr_dpb_luma(%d): %u \n"
            "bottom field: misr_dpb_chroma(%d): %u \n"
            "  bottom field: misr_opb_luma(%d): %u \n"
            "bottom field: misr_opb_chroma(%d): %u \n",
            m, (unsigned int)outputcrop_info->misr_info[1].misr_dpb_luma[m],
            m, (unsigned int)outputcrop_info->misr_info[1].misr_dpb_chroma[m],
            m, (unsigned int)outputcrop_info->misr_info[1].misr_opb_luma[m],
            m, (unsigned int)outputcrop_info->misr_info[1].misr_opb_chroma[m]);
        }
        DEBUG_PRINT_HIGH("================== End of output crop ===========");
    } else if (extra->eType == OMX_ExtraDataNone) {
        DEBUG_PRINT_HIGH("========== End of Terminator ===========");
    } else {
        DEBUG_PRINT_HIGH("======= End of Driver Extradata ========");
    }
}
