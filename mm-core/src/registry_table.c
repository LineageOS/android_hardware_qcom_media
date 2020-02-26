/*--------------------------------------------------------------------------
Copyright (c) 2018-2019 The Linux Foundation. All rights reserved.

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
/*============================================================================
                            O p e n M A X   w r a p p e r s
                             O p e n  M A X   C o r e

 This module contains the registry table for the OpenMAX core.

*//*========================================================================*/

#include "qc_omx_core.h"

omx_core_cb_type core[] =
{
    //Common entries
    OMX_REGISTRY_ENTRY("OMX.qcom.video.decoder.avc", "libOmxVdec.so", "video_decoder.avc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.decoder.avc.dsmode", "libOmxVideoDSMode.so", "video_decoder.avc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.file.muxer", "libOmxMux.so", "container_muxer.mp2"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.decoder.hevc", "libOmxVdec.so", "video_decoder.hevc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.decoder.vp8", "libOmxVdec.so", "video_decoder.vp8"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.decoder.vp9", "libOmxVdec.so", "video_decoder.vp9"),
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.mpeg4sw", "libOmxSwVdec.so", "video_decoder.mpeg4"),
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.divxsw", "libOmxSwVdec.so", "video_decoder.divx"),
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.divx4sw", "libOmxSwVdec.so", DIVX4_MIME),
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.h263sw", "libOmxSwVdec.so", "video_decoder.h263"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.mpeg4sw", "libOmxSwVencMpeg4.so", "video_encoder.mpeg4"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.h263sw", "libOmxSwVencMpeg4.so", "video_encoder.h263"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.avc", "libOmxVenc.so", "video_encoder.avc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.vp8", "libOmxVenc.so", "video_encoder.vp8"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.hevc", "libOmxVenc.so", "video_encoder.hevc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.hevc.cq", "libOmxVenc.so", "video_encoder.hevc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.heic", "libOmxVenc.so", "image_encoder.heic"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.Qcelp13", "libOmxQcelp13Dec.so", "audio_decoder.Qcelp13"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.evrc", "libOmxEvrcDec.so", "audio_decoder.evrc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.wma", "libOmxWmaDec.so", "audio_decoder.wma"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.wma10Pro", "libOmxWmaDec.so", "audio_decoder.wma"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.alac", "libOmxAlacDec.so", "audio_decoder.alac"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.ape", "libOmxApeDec.so", "audio_decoder.ape"),
    OMX_REGISTRY_ENTRY("OMX.qti.audio.decoder.dsd", "libOmxDsdDec.so", "audio_decoder.dsd"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.aac", "libOmxAacEnc.so", "audio_encoder.aac"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.qcelp13", "libOmxQcelp13Enc.so", "audio_encoder.qcelp13"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.evrc", "libOmxEvrcEnc.so", "audio_encoder.evrc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.amrnb", "libOmxAmrEnc.so", "audio_encoder.amrnb"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.aac", "libOmxAacDec.so", "audio_decoder.aac"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.multiaac", "libOmxAacDec.so", "audio_decoder.aac"),

#ifdef _NILE_
    //Entries specific to msmnile
    OMX_REGISTRY_ENTRY("OMX.qti.vdec.vpp", "libOmxVpp.so", "video_decoder.vpp"),
#endif //_NILE_

#if defined(_STEPPE_) || defined(_ATOLL_)
    //Entries specific to msmsteppe
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.tme", "libOmxVenc.so", "video_encoder.tme"),
    OMX_REGISTRY_ENTRY("OMX.qcom.video.encoder.tme.secure", "libOmxVenc.so", "video_encoder.tme"),
    OMX_REGISTRY_ENTRY("OMX.qti.vdec.vpp", "libOmxVpp.so", "video_decoder.vpp"),
#endif //_STEPPE_ || _ATOLL_

#if !defined(_ATOLL_) && !defined(_TRINKET_)
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.vc1sw", "libOmxSwVdec.so", "video_decoder.vc1"),
    OMX_REGISTRY_ENTRY("OMX.qti.video.decoder.wmvsw", "libOmxSwVdec.so", "video_decoder.vc1"),
#endif //_ATOLL_ || _TRINKET_
};

const unsigned int SIZE_OF_CORE = sizeof(core) / sizeof(omx_core_cb_type);


