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
// Linux framebuffer
int framebufferFileHandle = -1;
#endif

#define SLOW_DEBUGGING 0

#define ESUCCESS 0

#define FBIOGET_VSCREENINFO 0x4600
#define FBIOPUT_VSCREENINFO 0x4601
#define FB_ACTIVATE_NOW     0   /* set values immediately (or vbl)*/

static std::atomic<bool> exit_loop(false);
static void sigint_handler(int)
{    exit_loop = true;
}

void configureScreen(NDIlib_video_frame_v2_t *video_recv);
void showFrame(NDIlib_video_frame_v2_t *video_recv);
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

    NDIlib_recv_create_v3_t NDI_recv_create_desc = { p_sources[source_number], NDIlib_recv_color_format_BGRX_BGRA , NDIlib_recv_bandwidth_highest, false, "NDIRec" };

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
                showFrame(&video_recv);
                break;
#ifdef ENABLE_AUDIO
            case NDIlib_frame_type_audio:
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

void configureScreen(NDIlib_video_frame_v2_t *video_recv) {
    static bool configured = false;
    if (configured) return;
    configured = true;

#if 1
    // Linux

    struct fb_bitfield {
        uint32_t offset;           /* beginning of bitfield    */
        uint32_t length;           /* length of bitfield       */
        uint32_t msb_right;        /* != 0 : Most significant bit is */ 
                                   /* right */ 
    };

    struct framebuffer_screen_info {
        uint32_t xres;                  /* visible resolution */
        uint32_t yres;
        uint32_t xres_virtual;          /* virtual resolution */
        uint32_t yres_virtual;
        uint32_t xoffset;               /* offset from virtual to visible */
        uint32_t yoffset;               /* resolution */

        uint32_t bits_per_pixel;
        uint32_t grayscale;             /* !=0 Graylevels instead of colors */

        struct fb_bitfield red;         /* bitfield in fb mem if true color, */
        struct fb_bitfield green;       /* else only length is significant */
        struct fb_bitfield blue;
        struct fb_bitfield transp;      /* transparency */

        uint32_t nonstd;                /* !=0 Non standard pixel format */

        uint32_t activate;              /* see FB_ACTIVATE_x */

        uint32_t height;                /* height of picture in mm */
        uint32_t width;                 /* width of picture in mm */

        uint32_t accel_flags;           /* acceleration flags (hints) */

        /* Timing: All values in pixclocks, except pixclock (of course) */
        uint32_t pixclock;              /* pixel clock in ps (pico seconds) */
        uint32_t left_margin;           /* time from sync to picture */
        uint32_t right_margin;          /* time from picture to sync */
        uint32_t upper_margin;          /* time from sync to picture */
        uint32_t lower_margin;
        uint32_t hsync_len;             /* length of horizontal sync */
        uint32_t vsync_len;             /* length of vertical sync */
        uint32_t sync;                  /* see FB_SYNC_x */
        uint32_t vmode;                 /* see FB_VMODE_x */
        uint32_t reserved[6];           /* Reserved for future compatibility */
    };

    framebufferFileHandle = open("/dev/fb0", O_RDONLY);
    if (framebufferFileHandle == -1) {
        perror("cameracontroller");
        fprintf(stderr, "Could not open framebuffer.\n");
    }

    struct framebuffer_screen_info screenInfo;
    int error = xioctl(framebufferFileHandle, FBIOGET_VSCREENINFO, &screenInfo);
    if (error != ESUCCESS) {
        perror("cameracontroller");
        fprintf(stderr, "framebuffer read ioctl failed with error %d\n", error);
        exit(1);
    }
    screenInfo.xres_virtual = video_recv->xres;
    screenInfo.yres_virtual = video_recv->yres;

    screenInfo.activate = FB_ACTIVATE_NOW;
    error = xioctl(framebufferFileHandle, FBIOPUT_VSCREENINFO, &screenInfo);
    if (error != ESUCCESS) {
        perror("cameracontroller");
        fprintf(stderr, "framebuffer read ioctl failed with error %d\n", error);
        exit(1);
    }

#else
  // Mac

#endif

}

void showFrame(NDIlib_video_frame_v2_t *video_recv) {
    configureScreen(video_recv);

    lseek(framebufferFileHandle, 0, SEEK_SET);
    write(framebufferFileHandle, video_recv->p_data, (video_recv->xres * video_recv->yres));
}

static int xioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}
