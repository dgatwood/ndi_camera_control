#include <csignal>
#include <cstddef>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <dlfcn.h>

#include <Processing.NDI.Lib.h>

#if 1
#include <linux/kd.h>
#include <linux/vt.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <sys/mman.h>
#include <sys/user.h>

// Linux framebuffer
int framebufferFileHandle = -1;
struct fb_var_screeninfo initialFramebufferConfiguration;
struct fb_var_screeninfo framebufferActiveConfiguration;
struct fb_fix_screeninfo framebufferFixedConfiguration;
unsigned char *framebufferMemory = NULL;
unsigned char *framebufferBase = NULL;
unsigned char *framebufferActiveMemory = NULL;

int framebufferXRes = 0, framebufferYRes = 0, NDIXRes = 0, NDIYRes = 0;
double xScaleFactor = 0.0, yScaleFactor = 0.0;

#endif

#define SLOW_DEBUGGING 0

#if 0
#define ESUCCESS 0

#define FBIOGET_VSCREENINFO 0x4600
#define FBIOPUT_VSCREENINFO 0x4601
#define FB_ACTIVATE_NOW     0   /* set values immediately (or vbl)*/
#endif

#ifndef PAGE_SHIFT
        #define PAGE_SHIFT 12
#endif
#ifndef PAGE_SIZE
        #define PAGE_SIZE (1UL << PAGE_SHIFT)
#endif
#ifndef PAGE_MASK
        #define PAGE_MASK (~(PAGE_SIZE - 1))
#endif


static std::atomic<bool> exit_loop(false);
static void sigint_handler(int)
{    exit_loop = true;
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv);
bool drawFrame(NDIlib_video_frame_v2_t *video_recv);
static int xioctl(int fd, int request, void *arg);

int main(int argc, char *argv[]) {
    std::string ndi_path;
    if (argc < 2) {
        fprintf(stderr, "Usage: camera_control \"stream name\"\n");
        fprintf(stderr, "Known sources:\n");
    }
    char *stream_name = argv[1];

#if 1
    /* BEGIN CRAP: Everything about the code below is gross, but it's boilerplate code. */
    const char* p_NDI_runtime_folder = ::getenv("NDI_RUNTIME_DIR_V4");
    if (p_NDI_runtime_folder) {
        ndi_path = p_NDI_runtime_folder;
        ndi_path += "/libndi.dylib";
    } else {
        ndi_path = "/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/libndi.so.4"; // The standard versioning scheme on Linux based systems using sym links
    }

    // Try to load the library
    void *hNDILib = ::dlopen(ndi_path.c_str(), RTLD_LOCAL | RTLD_LAZY);

    // The main NDI entry point for dynamic loading if we got the library
    const NDIlib_v3* (*NDIlib_v3_load)(void) = NULL;
    if (hNDILib) {
        *((void**)&NDIlib_v3_load) = ::dlsym(hNDILib, "NDIlib_v3_load");
    }

    if (!NDIlib_v3_load) {
        printf("Please re-install the NewTek NDI Runtimes to use this application.");
        return 0;
    }
#endif

    // Lets get all of the DLL entry points
    const NDIlib_v3 *p_NDILib = NDIlib_v4_load();

    // We can now run as usual
    if (!p_NDILib->NDIlib_initialize())
    {    // Cannot run NDI. Most likely because the CPU is not sufficient (see SDK documentation).
        // you can check this directly with a call to NDIlib_is_supported_CPU()
        printf("Cannot run NDI.");
        return 0;
    }

    /* END CRAP: Everything about the code above is gross, but it's boilerplate code. */

    // Catch interrupt so that we can shut down gracefully
    signal(SIGINT, sigint_handler);

    // We first need to look for a source on the network
    const NDIlib_find_create_t NDI_find_create_desc = { true, NULL };

    // Create a finder
    NDIlib_find_instance_t pNDI_find = p_NDILib->NDIlib_find_create_v2(&NDI_find_create_desc);
    if (!pNDI_find) return 0;

    // We wait until there is at least one source on the network
    uint32_t no_sources = 0;
    const NDIlib_source_t *p_sources = NULL;
    while (!exit_loop && !no_sources)
    {    // Wait until the sources on the nwtork have changed
        p_NDILib->NDIlib_find_wait_for_sources(pNDI_find, 1000);
        p_sources = p_NDILib->NDIlib_find_get_current_sources(pNDI_find, &no_sources);
    }

    // We need at least one source
    if (!p_sources) return 0;

    int source_number = -1;
    if (stream_name) {
        fprintf(stderr, "Searching for stream \"%s\"\n", stream_name);
    }
    for (int i = 0; i < no_sources; i++) {
        if (stream_name) {
            if (!strcmp(p_sources[i].p_ndi_name, stream_name)) {
#if SLOW_DEBUGGING
                fprintf(stderr, "Chose \"%s\"\n", p_sources[i].p_ndi_name);
#endif
                source_number = i;
                break;
#if SLOW_DEBUGGING
            } else {
                fprintf(stderr, "Not \"%s\"\n", p_sources[i].p_ndi_name);
#endif
            }
        } else {
            fprintf(stderr, "    \"%s\"\n", p_sources[i].p_ndi_name);
        }
    }
    if (source_number == -1) {
        printf("Could not find source.\n");
        exit(1);
    }

    NDIlib_recv_create_v3_t NDI_recv_create_desc = { p_sources[source_number], NDIlib_recv_color_format_BGRX_BGRA, NDIlib_recv_bandwidth_lowest, false, "NDIRec" };

    // Create the receiver
    NDIlib_recv_instance_t pNDI_recv = NDIlib_recv_create_v3(&NDI_recv_create_desc);

    if (!pNDI_recv)
    {    p_NDILib->NDIlib_find_destroy(pNDI_find);
        return 0;
    }

    // Destroy the NDI finder. We needed to have access to the pointers to p_sources[0]
    p_NDILib->NDIlib_find_destroy(pNDI_find);

    fprintf(stderr, "Ready.\n");

    while (!exit_loop) {
        NDIlib_video_frame_v2_t video_recv;
#ifdef ENABLE_AUDIO
        NDIlib_audio_frame_v3_t audio_recv;
#endif
        NDIlib_frame_type_e frameType =
        NDIlib_recv_capture_v3(pNDI_recv, &video_recv,
#ifdef ENABLE_AUDIO
               &audio_recv,
#else
               nullptr,
#endif
               nullptr, 1500);
        switch(frameType) {
            case NDIlib_frame_type_video:
#if SLOW_DEBUGGING
                fprintf(stderr, "Video frame\n");
#endif
                if (!drawFrame(&video_recv)) {
                    // The framebuffer configuration failed.  We can't do anything.
                    exit_loop = true;
                }
                NDIlib_recv_free_video_v2(pNDI_recv, &video_recv);
                break;
#ifdef ENABLE_AUDIO
            case NDIlib_frame_type_audio:
                NDIlib_recv_free_audio_v2(pNDI_recv, &audio_recv);
#endif
            default:
#if SLOW_DEBUGGING
                fprintf(stderr, "Unknown frame type %d.\n", frameType);
#endif
                true;
        }
    }

    if (pNDI_recv != nullptr) {
        // Destroy the receiver

        printf("Closing the connection.\n");
        p_NDILib->NDIlib_recv_connect(pNDI_recv, NULL);

        printf("Destroying the NDI receiver.\n");
        p_NDILib->NDIlib_recv_destroy(pNDI_recv);
        printf("The NDI receiver has been destroyed.\n");
    }

    // Not required, but nice
    p_NDILib->NDIlib_destroy();
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv) {
    static bool configured = false;
    if (configured) return true;
    configured = true;

#if 1
    // Read the current framebuffer settings so that we can restore them later.
    int framebufferMemoryOffset = 0;
    framebufferFileHandle = open("/dev/fb0", O_RDWR);
    if (framebufferFileHandle == -1) {
        perror("cameracontroller: open");
        goto fail;
    }
    if (ioctl(framebufferFileHandle, FBIOGET_VSCREENINFO, &initialFramebufferConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_VSCREENINFO");
        goto fail;
    }
    if (ioctl(framebufferFileHandle, FBIOGET_FSCREENINFO, &framebufferFixedConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_FSCREENINFO");
        goto fail;
    }
    if (framebufferFixedConfiguration.type != FB_TYPE_PACKED_PIXELS) {
        fprintf(stderr, "Error: Only packed pixel framebuffers are supported.\n");
        goto fail;
    }

    framebufferMemoryOffset = (unsigned long)(framebufferFixedConfiguration.smem_start) & (~PAGE_MASK);
    // framebufferMemory = (unsigned char *)mmap(NULL, framebufferFixedConfiguration.smem_len + framebufferMemoryOffset, PROT_READ | PROT_WRITE, MAP_SHARED, framebufferFileHandle, 0);
    framebufferMemory = (unsigned char *)mmap(NULL, framebufferFixedConfiguration.smem_len + framebufferMemoryOffset, PROT_READ | PROT_WRITE, MAP_SHARED, framebufferFileHandle, 0);
    if ((long)framebufferMemory == -1L) {
        perror("cameracontroller: mmap");
        goto fail;
    }

#if 0
    // move viewport to upper left corner
    if (initialFramebufferConfiguration.xoffset != 0 || initialFramebufferConfiguration.yoffset != 0) {
        fprintf(stderr, "Shifting framebuffer offset from (%d, %d) to (0, 0)\n", initialFramebufferConfiguration.xoffset, initialFramebufferConfiguration.yoffset);
        initialFramebufferConfiguration.xoffset = 0;
        initialFramebufferConfiguration.yoffset = 0;
        if (ioctl(framebufferFileHandle, FBIOPAN_DISPLAY, &initialFramebufferConfiguration) == -1) {
                perror("cameracontroller: FBIOPAN_DISPLAY");
                munmap(framebufferMemory, framebufferFixedConfiguration.smem_len);
                goto fail;
        }
    }
#endif

    framebufferActiveConfiguration = initialFramebufferConfiguration;
    framebufferActiveConfiguration.xoffset = 0;
    framebufferActiveConfiguration.yoffset = 0;
    // framebufferActiveConfiguration.xoffset = 0;
    // framebufferActiveConfiguration.yoffset = initialFramebufferConfiguration.yres - 1;
    // framebufferActiveConfiguration.xres = video_recv->xres;
    // framebufferActiveConfiguration.yres = video_recv->yres;

    // framebufferActiveConfiguration.bits_per_pixel = 32;
    framebufferXRes = framebufferActiveConfiguration.xres;
    framebufferYRes = framebufferActiveConfiguration.yres;
    NDIXRes = video_recv->xres;
    NDIYRes = video_recv->yres;

    xScaleFactor = ((double)(framebufferXRes) / (double)(NDIXRes));
    yScaleFactor = ((double)(framebufferYRes) / (double)(NDIYRes));

    fprintf(stderr, "NDI Xres: %d, Yres: %d\n", video_recv->xres, video_recv->yres);
    fprintf(stderr, "Xres: %d, Yres: %d, bpp: %d\n", framebufferActiveConfiguration.xres, framebufferActiveConfiguration.yres, framebufferActiveConfiguration.bits_per_pixel);

    if (ioctl(framebufferFileHandle, FBIOPAN_DISPLAY, &framebufferActiveConfiguration) == -1) {
        perror("cameracontroller: FBIOPAN_DISPLAY (2)");
        munmap(framebufferMemory, framebufferFixedConfiguration.smem_len);
        goto fail;
    }

    framebufferBase = framebufferMemory + framebufferMemoryOffset;
    framebufferActiveMemory = framebufferMemory + framebufferMemoryOffset + (initialFramebufferConfiguration.yres * initialFramebufferConfiguration.xres * (initialFramebufferConfiguration.bits_per_pixel / 8));
    return true;

  fail:
    if (ioctl(framebufferFileHandle, FBIOPUT_VSCREENINFO, &initialFramebufferConfiguration) == -1) {
        perror("cameracontroller: FBIOPUT_VSCREENINFO");
    }
    if (ioctl(framebufferFileHandle, FBIOGET_FSCREENINFO, &framebufferFixedConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_FSCREENINFO");
    }
    return false;

#else
  // Mac

#endif

}

uint16_t convert_sample(uint32_t sample) {
    // AA RR GG BB

    // printf("Sample: 0x%x\n", sample);
    uint8_t r = (sample >> 16) & 0xff;
    uint8_t g = (sample >>  8) & 0xff;
    uint8_t b = (sample >>  0) & 0xff;
    // uint8_t g = 0;
    // uint8_t b = 0;
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (g >> 3);

    // return 0b1111100000010000;
    //       ..rrrrrggggggbbbbb;
}

// Takes frame coordinate and returns index into framebuffer.
uint32_t scaledRow(uint32_t y) {
    return y * yScaleFactor;
}

// Takes frame coordinate and returns index into framebuffer.
uint32_t scaledColumn(uint32_t x) {
    return x * xScaleFactor;
}

// Takes framebuffer coordinate and returns index into frame.
uint32_t scaledPosition(uint32_t x, uint32_t y) {
    static int maxPos = 0;
    if (x > NDIXRes) return 0;
    if (y > NDIYRes) return 0;
    // uint32_t scaledX = x / xScaleFactor;
    // uint32_t scaledY = y / yScaleFactor;
    // return scaledX + (scaledY * NDIXRes);
    return (y * NDIXRes) + x;
}

bool drawFrame(NDIlib_video_frame_v2_t *video_recv) {
    if (!configureScreen(video_recv)) {
        return false;
    }
    // memset(framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));
    // memset(framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));

#if 1
    //bcopy(video_recv->p_data, framebufferBase, (video_recv->xres * video_recv->yres * 2));
    uint32_t *inBuf = (uint32_t *)video_recv->p_data;
    uint16_t *outBuf = (uint16_t *)framebufferBase;
    for (int y = 0; y < video_recv->yres; y++) {
        int minRow = scaledRow(y);
        int maxRow = scaledRow(y+1) - 1;
        for (int x = 0; x < video_recv->xres; x++) {
            uint32_t *inPos = &inBuf[(y * video_recv->xres) + x];
            for (int outX = scaledColumn(x); outX < scaledColumn(x+1); outX++) {
                uint16_t *outPos = &outBuf[(minRow * framebufferXRes) + outX];
                *outPos = convert_sample(*inPos);
            }
        }
        for (int row = minRow + 1; row <= maxRow; row++) {
            bcopy(&outBuf[(minRow * framebufferXRes)],
                  &outBuf[(row * framebufferXRes)], framebufferXRes * 2);
        }
    }
#else
    for (int y = 0; y < framebufferYRes; y++) {
        for (int x = 0; x < framebufferXRes; x++) {
            uint32_t pos = (y * framebufferXRes) + x;
            uint16_t *outPos = (uint16_t *)&framebufferBase[(pos * 2)];
            uint32_t *inPos = (uint32_t *)&video_recv->p_data[(scaledPosition(x, y) * 4)];
            *outPos = convert_sample(*inPos);
        }
    }
#endif

    // bcopy(video_recv->p_data, framebufferBase, (video_recv->xres * video_recv->yres * 4));
// framebufferActiveMemory
// framebufferBase

    // lseek(framebufferFileHandle, 0, SEEK_SET);
    // write(framebufferFileHandle, video_recv->p_data, (video_recv->xres * video_recv->yres));
    return true;
}

static int xioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (r == -1 && EINTR == errno);
    return r;
}


void cleanupFrameBuffer(void) {
    if (ioctl(framebufferFileHandle, FBIOPUT_VSCREENINFO, &initialFramebufferConfiguration) == -1) {
        fprintf(stderr, "Ioctl FBIOPUT_VSCREENINFO error.\n");
    }
    if (ioctl(framebufferFileHandle, FBIOGET_FSCREENINFO, &framebufferFixedConfiguration) == -1) {
        fprintf(stderr, "Ioctl FBIOGET_FSCREENINFO.\n");
    }
    munmap(framebufferMemory, framebufferFixedConfiguration.smem_len);
    close(framebufferFileHandle);
}

