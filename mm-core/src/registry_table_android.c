/*--------------------------------------------------------------------------
Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.

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

    //------------- Audio HW components -----------------------------------------------------------
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.Qcelp13", "libOmxQcelp13Dec.so", "audio_decoder.Qcelp13"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.evrc", "libOmxEvrcDec.so", "audio_decoder.evrc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.wma", "libOmxWmaDec.so", "audio_decoder.wma"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.wma10Pro", "libOmxWmaDec.so", "audio_decoder.wma"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.wmaLossLess", "libOmxWmaDec.so", "audio_decoder.wma"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.amrwbplus", "libOmxAmrwbplusDec.so", "audio_decoder.awbplus"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.alac", "libOmxAlacDec.so", "audio_decoder.alac"),
#ifndef ENABLE_OMX_SW_CODECS
    OMX_REGISTRY_ENTRY("OMX.qti.audio.decoder.alac.sw", "libOmxAlacDecSw.so", "audio_decoder.alac"),
#endif
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.ape", "libOmxApeDec.so", "audio_decoder.ape"),
#ifndef ENABLE_OMX_SW_CODECS
    OMX_REGISTRY_ENTRY("OMX.qti.audio.decoder.ape.sw", "libOmxApeDecSw.so", "audio_decoder.ape"),
#endif
    OMX_REGISTRY_ENTRY("OMX.qti.audio.decoder.dsd", "libOmxDsdDec.so", "audio_decoder.dsd"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.aac", "libOmxAacEnc.so", "audio_encoder.aac"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.qcelp13", "libOmxQcelp13Enc.so", "audio_encoder.qcelp13"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.evrc", "libOmxEvrcEnc.so", "audio_encoder.evrc"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.amrnb", "libOmxAmrEnc.so", "audio_encoder.amrnb"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.multiaac", "libOmxAacDec.so", "audio_decoder.aac"),

#ifdef _EN_ADDTNL_CDCS_
#ifndef ENABLE_OMX_SW_CODECS
    OMX_REGISTRY_ENTRY("OMX.qti.audio.decoder.mpegh", "libOmxMpeghDecSw.so", "audio_decoder.mpegh"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.encoder.mpegh", "libOmxMpeghEncSw.so", "audio_encoder.mpegh"),
#endif
#endif //_EN_ADDTNL_CDCS_

    // HACK: Hidden components marker
    OMX_REGISTRY_ENTRY("OMX.QCOM.CUST.COMP.START", NULL, NULL),

    // Hidden components
    OMX_REGISTRY_ENTRY("OMX.qcom.file.muxer", "libOmxMux.so", "container_muxer.mp2"),
    OMX_REGISTRY_ENTRY("OMX.qcom.audio.decoder.aac", "libOmxAacDec.so", "audio_decoder.aac"),

};
const unsigned int SIZE_OF_CORE = sizeof(core) / sizeof(omx_core_cb_type);
