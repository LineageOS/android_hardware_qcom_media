/*
 * Extrapolated / reversed header for PQ encoding
 */

#ifndef GPUSTATS_H
#define GPUSTATS_H

#define ADAPTIVE_QP 1

enum color_compression_format {
    LINEAR_NV12,
    UBWC_NV12
};

enum perf_hint {
    LOW    = 1,
    NORMAL = 2,
    HIGH   = 3,
};

struct gpu_stats_lib_buffer_params_t {
    int fd;
    int data_offset;
    int alloc_len;
    int filled_len;
};

struct gpu_stats_lib_caps_t {
    unsigned int max_width;
    unsigned int max_height;
    int color_formats;
    int max_mb_per_sec;
};

struct something /* maybe adaptive_qp_params or so? */ {
    bool roi_enabled;
    int pq_enabled;
    float gain;
    float offset;
    int minDeltaQPlimit;
    int maxDeltaQPlimit;
};

struct gpu_stats_lib_input_config {
    struct something a_qp;
    int algo;
    int height;
    int width;
    int mb_height;
    int mb_width;
    int stride;
};

enum gpu_stats_lib_op_status { /* somethings supposed to go in here but ¯\_(ツ)_/¯ */ };

#endif /* GPUSTATS_H */
