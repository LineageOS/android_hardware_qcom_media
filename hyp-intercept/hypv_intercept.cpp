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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <errno.h>

#include "hypv_intercept.h"
#include "vidc_debug.h"
#ifdef _ANDROID_
#define LOG_TAG "HYPV-INTERCEPT"
extern "C" {
#include<utils/Log.h>
}
#else
#define ALOGE(fmt, args...) fprintf(stderr, fmt, ##args)
#endif /* _ANDROID_ */

typedef V4L2FE_HANDLE (*v4l2fe_open_func)(const char*, int, v4l2fe_callback_t*);
typedef int (*v4l2fe_ioctl_func)(V4L2FE_HANDLE, int, void*);
typedef int (*v4l2fe_close_func)(V4L2FE_HANDLE);

#define MAX_V4L2_HANDLE_STORAGE     16
#define HYPV_HANDLE_SIGNATURE       0x2bcd0000
#define HYPV_HANDLE_SIGNATURE_MASK  0xffff0000
#define HYPV_HANDLE_MASK            0x0000ffff


enum hypv_which_build_t
{
    HYPV_NOT_INITIAILIZED = 0xdeadbeef,
    HYPV_HYPERVISOR_BUILD = 1
};

static void *glib_handle = NULL;
static v4l2fe_open_func v4l2fe_open = NULL;
static v4l2fe_ioctl_func v4l2fe_ioctl = NULL;
static v4l2fe_close_func v4l2fe_close = NULL;
static hypv_which_build_t ghypv_which_build = HYPV_NOT_INITIAILIZED;
static pthread_mutex_t g_v4l2_handle_storage_lock = PTHREAD_MUTEX_INITIALIZER;
static V4L2FE_HANDLE g_v4l2_handle_storage[MAX_V4L2_HANDLE_STORAGE];
static int g_v4l2_handle_count  = 0;
static int g_hypv_init_count = 0;

#define IS_HYPERVISOR_VIDEO(fd) ((ghypv_which_build == HYPV_HYPERVISOR_BUILD) &&\
                                 ((fd & HYPV_HANDLE_SIGNATURE_MASK)==HYPV_HANDLE_SIGNATURE))

static int add_to_v4l2_handle_storage(V4L2FE_HANDLE handle_to_store)
{
    int rc = 0;

    pthread_mutex_lock(&g_v4l2_handle_storage_lock);
    if (g_v4l2_handle_count >= MAX_V4L2_HANDLE_STORAGE) {
        DEBUG_PRINT_ERROR("reached max handle count");
        rc = -1;
    } else {
        int i;

        for(i = 0; i < MAX_V4L2_HANDLE_STORAGE; i++) {
            if (g_v4l2_handle_storage[i] == 0) {
                g_v4l2_handle_storage[i] = handle_to_store;
                rc = i;
                g_v4l2_handle_count++;
                break;
            }
        }
        if (i >= MAX_V4L2_HANDLE_STORAGE) {
            DEBUG_PRINT_ERROR("failed to find empty slot");
            rc = -1;
        }
    }

    pthread_mutex_unlock(&g_v4l2_handle_storage_lock);

    return rc;
}

int hypv_init(void)
{
    int rc = 0;

    pthread_mutex_lock(&g_v4l2_handle_storage_lock);

    if (ghypv_which_build == HYPV_NOT_INITIAILIZED) {
        glib_handle = dlopen("libgioctl.so", RTLD_NOW);
        if (glib_handle == NULL) {
            DEBUG_PRINT_ERROR("failed to open gioctl lib");
            rc = -1;
        } else {
            ghypv_which_build = HYPV_HYPERVISOR_BUILD;
            g_v4l2_handle_count = 0;
            memset(g_v4l2_handle_storage, 0, sizeof(V4L2FE_HANDLE)*MAX_V4L2_HANDLE_STORAGE);
            v4l2fe_open = (v4l2fe_open_func)dlsym(glib_handle, "v4l2fe_open");
            if (v4l2fe_open == NULL) {
                DEBUG_PRINT_ERROR("failed to get v4l2fe_open handle");
                rc = -1;
            } else {
                v4l2fe_ioctl = (v4l2fe_ioctl_func)dlsym(glib_handle, "v4l2fe_ioctl");
                if (v4l2fe_ioctl == NULL) {
                    DEBUG_PRINT_ERROR("failed to get v4l2fe_ioctl handle");
                    rc = -1;
                } else {
                    v4l2fe_close = (v4l2fe_close_func)dlsym(glib_handle, "v4l2fe_close");
                    if (v4l2fe_close == 0) {
                        DEBUG_PRINT_ERROR("failed to get v4l2fe_close handle");
                        rc = -1;
                    }//v4l2fe_close
                } //v4l2fe_iocl
            } //v4l2fe_open
        } //glib_handle
    }//initialize

    if (rc == 0)
        g_hypv_init_count++;

    pthread_mutex_unlock(&g_v4l2_handle_storage_lock);

    return rc;
}

int hypv_open(const char *str, int flag, v4l2fe_callback_t* cb)
{
    int rc = 0;

    if (ghypv_which_build == HYPV_NOT_INITIAILIZED) {
        DEBUG_PRINT_ERROR("hypervisor not initialized");
        rc = -1;
    } else if (ghypv_which_build == HYPV_HYPERVISOR_BUILD) {
        V4L2FE_HANDLE v4l2_handle = v4l2fe_open(str, flag, cb);
        DEBUG_PRINT_INFO("v4l2fe_open handle=%p", v4l2_handle);
        if (v4l2_handle == NULL) {
            DEBUG_PRINT_ERROR("v4l2fe_open failed");
            rc = -1;
        } else {
            int fd = 0;

            fd = add_to_v4l2_handle_storage(v4l2_handle);
            if (fd < 0) {
                DEBUG_PRINT_ERROR("failed to store v4l2 handle");
                v4l2fe_close(v4l2_handle);
                rc = -1;
            } else {
                rc = (HYPV_HANDLE_SIGNATURE | fd);
            }
        }
    }

    return rc;
}

int hypv_ioctl(int fd, int cmd, void *data)
{
    int rc = 0;

    if (ghypv_which_build == HYPV_NOT_INITIAILIZED) {
        DEBUG_PRINT_ERROR("hypervisor not initialized");
        rc = -1;
    } else if (IS_HYPERVISOR_VIDEO(fd)) {
        int fd_index = fd & HYPV_HANDLE_MASK;
        if (fd_index >= MAX_V4L2_HANDLE_STORAGE) {
            DEBUG_PRINT_ERROR("invalid fd_index=%d", fd_index);
            rc = -1;
        }
        rc = v4l2fe_ioctl(g_v4l2_handle_storage[fd_index], cmd, data);
        DEBUG_PRINT_INFO("hyp ioctl: fd=%d, fd_index=%d, cmd=0x%x, data=0x%p, rc =%d",
                          fd, fd_index, cmd, data, rc);
    } else {
        DEBUG_PRINT_ERROR("native ioctl: fd=%d, cmd=0x%x, data=0x%p", fd, cmd, data);
        rc = ioctl(fd, cmd, data);
    }

    return rc;
}

int hypv_close(int fd)
{
    int rc = 0;

    if (ghypv_which_build == HYPV_NOT_INITIAILIZED) {
        DEBUG_PRINT_ERROR("hypervisor not initialized");
        rc = -1;
    } else if (IS_HYPERVISOR_VIDEO(fd)) {
        int rc = 0;
        int fd_index = fd & HYPV_HANDLE_MASK;

        if ((fd_index >= MAX_V4L2_HANDLE_STORAGE) || (fd_index < 0)) {
            DEBUG_PRINT_ERROR("invalid fd=%d", fd_index);
            rc = -1;
        } else {
            int handle_count = 0;
            pthread_mutex_lock(&g_v4l2_handle_storage_lock);
            rc = v4l2fe_close(g_v4l2_handle_storage[fd_index]);
            g_v4l2_handle_storage[fd_index] = 0;
            g_v4l2_handle_count--;
            pthread_mutex_unlock(&g_v4l2_handle_storage_lock);
        }
    } else {
        rc = close(fd);
    }

    return rc;
}

int hypv_deinit()
{
    int rc = 0;

    if (ghypv_which_build == HYPV_NOT_INITIAILIZED) {
        DEBUG_PRINT_ERROR("hypervisor not initialized");
        rc = -1;
    } else {
        int init_count = 0;

        pthread_mutex_lock(&g_v4l2_handle_storage_lock);
        init_count = --g_hypv_init_count;
        pthread_mutex_unlock(&g_v4l2_handle_storage_lock);
        if (init_count == 0) {
            if (glib_handle != NULL) {
                dlclose(glib_handle);
                glib_handle = NULL;
            }
            ghypv_which_build = HYPV_NOT_INITIAILIZED;
        }
    }

    return rc;
}
