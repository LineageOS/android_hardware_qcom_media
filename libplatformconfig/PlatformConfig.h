/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////
#include <string>
#include <map>

#ifdef __cplusplus
    extern "C" {
#endif

namespace Platform {

typedef std::map<std::string, std::string> ConfigMap;

typedef enum {
    OK = 0,
    FAIL = 1,
} ConfigError_t;

typedef enum {
    vidc_dec_log_in = 0,
    vidc_dec_log_out,
    vidc_dec_hfr_fps,
    vidc_enc_log_in,
    vidc_enc_log_out,
    vidc_dec_conceal_color_8bit,
    vidc_dec_conceal_color_10bit,
    vidc_enc_csc_custom_matrix,
    vidc_dec_arb_mode_override,
} Config_t;

struct configStr {
    Config_t config;
    const char * name;
};

static const struct configStr configStrMap[] = {
    {vidc_dec_log_in, "vidc_dec_log_in"},
    {vidc_dec_log_out, "vidc_dec_log_out"},
    {vidc_dec_hfr_fps, "vidc_dec_hfr_fps"},
    {vidc_enc_log_in, "vidc_enc_log_in"},
    {vidc_enc_log_out, "vidc_enc_log_out"},
    {vidc_dec_conceal_color_8bit, "vidc_dec_conceal_color_8bit"},
    {vidc_dec_conceal_color_10bit, "vidc_dec_conceal_color_10bit"},
    {vidc_enc_csc_custom_matrix, "vidc_enc_csc_custom_matrix"},
    {vidc_dec_arb_mode_override, "vidc_dec_arb_mode_override"},
};

class Config {
    private:
        Config();
        Config& operator=(Config const&) {
            return *mInstance;
        }
        static Config* getInstance();

        ConfigMap mConfigMap;
        static Config* mInstance;

    public:
        static ConfigError_t getInt32(Config_t config, int32_t *value,
                const int32_t defaultValue);
};

}
#ifdef __cplusplus
}
#endif
#endif // __PLATFORM_CONFIG_H__
