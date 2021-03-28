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

#include <pthread.h>

#include <Processing.NDI.Lib.h>

// #define DEMO_MODE

// This must be set correctly in Linux, because the NDI library is bizarre.
#define NDI_LIBRARY_PATH "/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/libndi.so.4"

#ifdef __linux__
    #include <linux/kd.h>
    #include <linux/vt.h>
    #include <linux/fb.h>
    #include <linux/input.h>
    #include <sys/mman.h>
    #include <sys/user.h>

    #define I2C_ADDRESS 0x18

#else  // ! __linux__
    // Mac (partial support for testing).  Reads button values from files:
    //     /var/tmp/axis.0 through 2 for X/Y/Zoom values.
    //     /var/tmp/button.0 (set button) - button down if file exists.
    //     /var/tmp/button.1 through 6 - button down if file exists.

    #import <AppKit/AppKit.h>
    #import <CoreServices/CoreServices.h>
    #import <ImageIO/ImageIO.h>
#endif  // __linux__

#include "ioexpander.c"

int monitor_bytes_per_pixel = 4;
bool force_slow_path = false;  // For debugging.

/*
 * Controlled by the -f (--fast) flag.
 *
 * If true, this tool requests a low-quality stream (typically 720p).
 * If false, it requests a high-quality stream (typically 1080p).
 */
bool use_low_res_preview = false;

/* Enable debugging (controlled by the -d / --debug flag). */
bool enable_debugging = false;
/* Enable debugging (controlled by the -v / --verbose flag). */
bool enable_verbose_debugging = false;

#pragma mark - Constants and types


#ifdef __linux__
    #ifndef PAGE_SHIFT
        #define PAGE_SHIFT 12
    #endif
    #ifndef PAGE_SIZE
        #define PAGE_SIZE (1UL << PAGE_SHIFT)
    #endif
    #ifndef PAGE_MASK
        #define PAGE_MASK (~(PAGE_SIZE - 1))
    #endif
#endif

#define MAX_BUTTONS 6  // Theoretically, 9, but I don't want to build that much hardware.  Numbered 1 to 6.
#define BUTTON_SET 0   // If the set button is held down, we store a value for that button instead of retrieving it.

typedef struct {
    float xAxisPosition;
    float yAxisPosition;
    float zoomPosition;
    int storePositionNumber;    // Set button is down along with a number button (sent once/debounced).
    int retrievePositionNumber; // A number button is down by itself (sent once/debounced).
} motionData_t;

enum {
    kPTZAxisX = 1,
    kPTZAxisY,
    kPTZAxisZoom
};

#pragma mark - Globals

#if __linux__
    ioexpander_t *io_expander = NULL;

    // Linux framebuffer
    int g_framebufferFileHandle = -1;
    struct fb_var_screeninfo g_initialFramebufferConfiguration;
    struct fb_var_screeninfo g_framebufferActiveConfiguration;
    struct fb_fix_screeninfo g_framebufferFixedConfiguration;
    unsigned char *g_framebufferMemory = NULL;
    unsigned char *g_framebufferBase = NULL;
    unsigned char *g_framebufferActiveMemory = NULL;

    int g_framebufferXRes = 0, g_framebufferYRes = 0, g_NDIXRes = 0, g_NDIYRes = 0;
    double g_xScaleFactor = 0.0, g_yScaleFactor = 0.0;
#else  // ! __linux__
    NSWindow *g_mainWindow = nil;
    NSImageView *g_mainImageView = nil;
#endif  // __linux__

bool g_ptzEnabled = false;
#if __linux__
pthread_mutex_t g_motionMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;
#else
pthread_mutex_t g_motionMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER;
#endif

motionData_t g_motionData;

static std::atomic<bool> exit_loop(false);
static void sigint_handler(int)
{    exit_loop = true;
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv);
bool drawFrame(NDIlib_video_frame_v2_t *video_recv);
void *runPTZThread(void *argIgnored);
void sendPTZUpdates(NDIlib_recv_instance_t pNDI_recv);
void setMotionData(motionData_t newMotionData);
uint32_t find_named_source(const NDIlib_source_t *p_sources,
                           uint32_t no_sources,
                           char *stream_name,
                           bool use_fallback);

#ifdef DEMO_MODE
void demoPTZValues(void);
#endif

#ifdef __linux__
int pinNumberForAxis(int axis);
int pinNumberForButton(int button);
#endif  // __linux__

int main(int argc, char *argv[]) {
    char *stream_name = argv[argc - 1];
    if (argc < 2) {
        fprintf(stderr, "Usage: camera_control \"stream name\"\n");
        fprintf(stderr, "Known sources:\n");
        stream_name = NULL;
    }
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--debug")) {
            fprintf(stderr, "Enabling debugging (slow).\n");
            enable_debugging = true;
        }
        if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) {
            fprintf(stderr, "Enabling verbose debugging (slow).\n");
            enable_verbose_debugging = true;
        }
        if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--fast")) {
            fprintf(stderr, "Using low-res mode.\n");
            use_low_res_preview = true;
        }
    }

#ifdef __linux__
#ifndef DEMO_MODE
    io_expander = newIOExpander(I2C_ADDRESS, 0, -1, 0, false);
    if (io_expander) {
        for (int pin = kPTZAxisX; pin <= kPTZAxisZoom; pin++) {
            ioe_set_mode(io_expander, pinNumberForAxis(pin), PIN_MODE_ADC, false, false);
        }
        // Button 0 is BUTTON_SET.  Configure it like the other buttons.
        for (int button = BUTTON_SET; button <= MAX_BUTTONS; button++) {
            ioe_set_mode(io_expander, pinNumberForButton(button), PIN_MODE_PU, false, false);
        }
    }
#endif
#endif  // __linux__

    pthread_t motionThread;
    pthread_create(&motionThread, NULL, runPTZThread, NULL);

#ifndef __linux__
    dispatch_queue_t queue = dispatch_queue_create("ndi run loop", 0);
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.0 * NSEC_PER_SEC)), queue, ^{
#endif  // __linux__

        std::string ndi_path;

        // Try to look up the NDI SDK shared library.
        const char* p_NDI_runtime_folder = ::getenv("NDI_RUNTIME_DIR_V4");
        if (p_NDI_runtime_folder) {
            ndi_path = p_NDI_runtime_folder;
            ndi_path += "/libndi.dylib";
        } else {
#ifdef __linux__
            ndi_path = NDI_LIBRARY_PATH;
#else  // ! __linux__
            ndi_path = "libndi.4.dylib";
#endif  // __linux__
        }

        // Try to load the NDI SDK shared library (boilerplate NDI SDK code).
        void *hNDILib = ::dlopen(ndi_path.c_str(), RTLD_LOCAL | RTLD_LAZY);

        // Dynamically look up the shared library's initialization function (boilerplate NDI SDK code).
        const NDIlib_v3* (*NDIlib_v3_load)(void) = NULL;
        if (hNDILib) {
            *((void**)&NDIlib_v3_load) = ::dlsym(hNDILib, "NDIlib_v3_load");
        }

        if (!NDIlib_v3_load) {
            printf("Please re-install the NewTek NDI Runtimes to use this application.");
            exit(0);
        }

        // Run the shared library initialization function (boilerplate NDI SDK code).
        const NDIlib_v3 *p_NDILib = NDIlib_v4_load();

        // Initialize the NDI library (boilerplate NDI SDK code).
        if (!p_NDILib->NDIlib_initialize())
        {
            // NDI is not supported- nost likely because the CPU is not sufficient (see SDK documentation).
            // You can check this directly by calling NDIlib_is_supported_CPU().
            printf("Cannot run NDI.");
            exit(0);
        }

        // Catch SIGINT so that this tool can close NDI streams properly if the user presses control-C.
        signal(SIGINT, sigint_handler);
        signal(SIGTERM, sigint_handler);

        // First, search for NDI sources on the network.
        const NDIlib_find_create_t NDI_find_create_desc = { true, NULL };
        NDIlib_find_instance_t pNDI_find = p_NDILib->NDIlib_find_create_v2(&NDI_find_create_desc);
        if (!pNDI_find) exit(0);

        // Wait until at least one source is found, or one second, whichever is longer.
        uint32_t no_sources = 0;
        const NDIlib_source_t *p_sources = NULL;
        int source_number = -1;
        while (!exit_loop && source_number == -1) {
            // Wait until the sources on the network have changed
            p_NDILib->NDIlib_find_wait_for_sources(pNDI_find, 1000000);
            p_sources = p_NDILib->NDIlib_find_get_current_sources(pNDI_find, &no_sources);

            // If the user provided the name of a stream to display, search for it specifically.
            // Otherwise, just show a list of valid sources and exit.  Either way, iterate
            // through the sources.
            if (stream_name) {
                fprintf(stderr, "Searching for stream \"%s\"\n", stream_name);
            }
            source_number = find_named_source(p_sources, no_sources, stream_name, false);
            if (stream_name != NULL && source_number == -1) {
                source_number = find_named_source(p_sources, no_sources, stream_name, true);
            }
        }

        // If the user pressed control-C this early, exit immediately.
        if (!p_sources) exit(0);

        if (source_number == -1) {
            printf("Could not find source.\n");
            exit(1);
        }

        NDIlib_recv_create_v3_t NDI_recv_create_desc = {
                p_sources[source_number],
                NDIlib_recv_color_format_BGRX_BGRA,
                use_low_res_preview ? NDIlib_recv_bandwidth_lowest : NDIlib_recv_bandwidth_highest,
                false,
                "NDIRec"
        };

        // Create the receiver
        NDIlib_recv_instance_t pNDI_recv = NDIlib_recv_create_v3(&NDI_recv_create_desc);

        if (!pNDI_recv)
        {    p_NDILib->NDIlib_find_destroy(pNDI_find);
            exit(0);
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
                    if (enable_debugging) {
                        fprintf(stderr, "Video frame\n");
                    }
                    if (!drawFrame(&video_recv)) {
                        // The framebuffer configuration failed.  We can't do anything.
                        exit_loop = true;
                    }
                    NDIlib_recv_free_video_v2(pNDI_recv, &video_recv);
                    break;
                case NDIlib_frame_type_status_change:
                    g_ptzEnabled = NDIlib_recv_ptz_is_supported(pNDI_recv);
                    break;
#ifdef ENABLE_AUDIO
                case NDIlib_frame_type_audio:
                    // Do something with the audio here.
                    NDIlib_recv_free_audio_v2(pNDI_recv, &audio_recv);
#endif
                default:
                    if (enable_debugging) {
                        fprintf(stderr, "Unknown frame type %d.\n", frameType);
                    }
            }
            if (g_ptzEnabled) {
                sendPTZUpdates(pNDI_recv);
            } else {
                if (enable_debugging) {
                    fprintf(stderr, "PTZ Disabled\n");
                }
            }
        }

        if (pNDI_recv != nullptr) {
            // Clean up the NDI receiver (stops packet transmission).

            printf("Closing the connection.\n");
            p_NDILib->NDIlib_recv_connect(pNDI_recv, NULL);

            printf("Destroying the NDI receiver.\n");
            p_NDILib->NDIlib_recv_destroy(pNDI_recv);
            printf("The NDI receiver has been destroyed.\n");
        }

        // Clean up the library as a whole.
        p_NDILib->NDIlib_destroy();

#ifndef __linux__
        dispatch_async(dispatch_get_main_queue(), ^{
            CFRunLoopStop(CFRunLoopGetMain());
        });
    });
    NSApplicationLoad();
    CFRunLoopRun();
#endif
}

void truncate_name_before_ip(char *name) {
    for (char *pos = &name[strlen(name) - 1] ; pos >= name; pos--) {
        if (*pos == ',') {
            *pos = '\0';
            break;
        }
    }
}

bool source_name_compare(const char *name1, const char *name2, bool use_fallback) {
    if (!use_fallback) {
        return !strcmp(name1, name2);
    }

    char *truncname1 = NULL, *truncname2 = NULL;
    asprintf(&truncname1, "%s", name1);
    asprintf(&truncname2, "%s", name2);
    truncate_name_before_ip(truncname1);
    truncate_name_before_ip(truncname2);

fprintf(stderr, "CMP \"%s\" ?= \"%s\"\n", truncname1, truncname2);
    return !strcmp(truncname1, truncname2);
}

uint32_t find_named_source(const NDIlib_source_t *p_sources,
                           uint32_t no_sources,
                           char *stream_name,
                           bool use_fallback) {
    for (int i = 0; i < no_sources; i++) {
        if (stream_name) {
            if (source_name_compare(p_sources[i].p_ndi_name, stream_name, use_fallback)) {
                if (enable_debugging) {
                    fprintf(stderr, "Chose \"%s\"\n", p_sources[i].p_ndi_name);
                }
                return i;
            } else if (enable_debugging) {
                fprintf(stderr, "Not \"%s\"\n", p_sources[i].p_ndi_name);
            }
        } else {
            fprintf(stderr, "    \"%s\"\n", p_sources[i].p_ndi_name);
        }
    }
    return -1;
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv) {
    static bool configured = false;
    if (configured) return true;
    configured = true;

#ifdef __linux__
    // Read the current framebuffer settings so that we can restore them later.
    int framebufferMemoryOffset = 0;
    g_framebufferFileHandle = open("/dev/fb0", O_RDWR);
    if (g_framebufferFileHandle == -1) {
        perror("cameracontroller: open");
        goto fail;
    }
    if (ioctl(g_framebufferFileHandle, FBIOGET_VSCREENINFO, &g_initialFramebufferConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_VSCREENINFO");
        goto fail;
    }
    if (ioctl(g_framebufferFileHandle, FBIOGET_FSCREENINFO, &g_framebufferFixedConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_FSCREENINFO");
        goto fail;
    }
    if (g_framebufferFixedConfiguration.type != FB_TYPE_PACKED_PIXELS) {
        fprintf(stderr, "Error: Only packed pixel framebuffers are supported.\n");
        goto fail;
    }

    framebufferMemoryOffset = (unsigned long)(g_framebufferFixedConfiguration.smem_start) & (~PAGE_MASK);
    g_framebufferMemory = (unsigned char *)mmap(NULL, g_framebufferFixedConfiguration.smem_len + framebufferMemoryOffset, PROT_READ | PROT_WRITE, MAP_SHARED, g_framebufferFileHandle, 0);
    if ((long)g_framebufferMemory == -1L) {
        perror("cameracontroller: mmap");
        goto fail;
    }

    g_framebufferActiveConfiguration = g_initialFramebufferConfiguration;
    g_framebufferActiveConfiguration.xoffset = 0;
    g_framebufferActiveConfiguration.yoffset = 0;

    monitor_bytes_per_pixel = g_framebufferActiveConfiguration.bits_per_pixel / 8;
    g_framebufferXRes = g_framebufferActiveConfiguration.xres;
    g_framebufferYRes = g_framebufferActiveConfiguration.yres;
    g_NDIXRes = video_recv->xres;
    g_NDIYRes = video_recv->yres;

    g_xScaleFactor = ((double)(g_framebufferXRes) / (double)(g_NDIXRes));
    g_yScaleFactor = ((double)(g_framebufferYRes) / (double)(g_NDIYRes));

    fprintf(stderr, "NDI Xres: %d, Yres: %d\n", video_recv->xres, video_recv->yres);
    fprintf(stderr, "Xres: %d, Yres: %d, bpp: %d\n", g_framebufferActiveConfiguration.xres,
            g_framebufferActiveConfiguration.yres, g_framebufferActiveConfiguration.bits_per_pixel);

    if (ioctl(g_framebufferFileHandle, FBIOPAN_DISPLAY, &g_framebufferActiveConfiguration) == -1) {
        perror("cameracontroller: FBIOPAN_DISPLAY (2)");
        munmap(g_framebufferMemory, g_framebufferFixedConfiguration.smem_len);
        goto fail;
    }

    g_framebufferBase = g_framebufferMemory + framebufferMemoryOffset;
    g_framebufferActiveMemory = g_framebufferMemory + framebufferMemoryOffset + (g_initialFramebufferConfiguration.yres * g_initialFramebufferConfiguration.xres * (g_initialFramebufferConfiguration.bits_per_pixel / 8));
    return true;

  fail:
    if (ioctl(g_framebufferFileHandle, FBIOPUT_VSCREENINFO, &g_initialFramebufferConfiguration) == -1) {
        perror("cameracontroller: FBIOPUT_VSCREENINFO");
    }
    if (ioctl(g_framebufferFileHandle, FBIOGET_FSCREENINFO, &g_framebufferFixedConfiguration) == -1) {
        perror("cameracontroller: FBIOGET_FSCREENINFO");
    }
    return false;

#else  // ! __linux__
    // Mac
    dispatch_sync(dispatch_get_main_queue(), ^{
        NSRect frame = NSMakeRect(0, 0, video_recv->xres, video_recv->yres);
        g_mainWindow = [[NSWindow alloc] initWithContentRect:frame
                                                   styleMask:NSWindowStyleMaskTitled
                                                     backing:NSBackingStoreBuffered
                                                       defer:NO];
        [g_mainWindow setBackgroundColor:[NSColor whiteColor]];
        g_mainImageView = [[NSImageView alloc] initWithFrame:frame];
        [[g_mainWindow contentView] addSubview:g_mainImageView];
        [g_mainWindow makeKeyAndOrderFront:g_mainWindow];
    });

    return true;
#endif
}

#ifdef __linux__
    uint16_t convert_sample_to_16bpp(uint32_t sample) {
        // Input:  0xAA RR GG BB          // 32-bit LE integer: 8 bits each for ARGB (alpha high).
        // Output: 0brrrrr gggggg bbbbb;  // 16-bit LE integer: 5 R, 6 G, 5 B (red high).

        uint8_t r = (sample >> 16) & 0xff;
        uint8_t g = (sample >>  8) & 0xff;
        uint8_t b = (sample >>  0) & 0xff;
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (g >> 3);

    }

    // This is a really weak scaling algorithm, intended to be as fast as possible, to leave
    // as much CPU as possible free for decoding whatever crazy resolution or profile of H.264
    // (or worse, H.265) the camera might throw in our direction.  Experimentally, the overhead
    // of 32-bit to 16-bit conversion plus the overhead of doing even this minimal conversion
    // exceeds what the Raspberry Pi 4 can handle at 1080p, at least with the vc4-fkms-v3d
    // driver enabled, so if you want to add a better scaling algorithm, it needs to be
    // configurable.

    // Takes frame coordinate and returns index into framebuffer.
    uint32_t scaledRow(uint32_t y) {
        return y * g_yScaleFactor;
    }

    // Takes frame coordinate and returns index into framebuffer.
    uint32_t scaledColumn(uint32_t x) {
        return x * g_xScaleFactor;
    }

    // Takes framebuffer coordinate and returns index into frame.
    uint32_t scaledPosition(uint32_t x, uint32_t y) {
        static int maxPos = 0;
        if (x > g_NDIXRes) return 0;
        if (y > g_NDIYRes) return 0;
        return (y * g_NDIXRes) + x;
    }

    bool drawFrame(NDIlib_video_frame_v2_t *video_recv) {
        int zero = 0;
        ioctl(g_framebufferFileHandle, FBIO_WAITFORVSYNC, &zero);  // Should work, but doesn't.

        if (!configureScreen(video_recv)) {
            return false;
        }
        if (g_xScaleFactor == 1.0 && g_yScaleFactor == 1.0 && monitor_bytes_per_pixel == 4 && !force_slow_path) {
            if (enable_debugging) {
                fprintf(stderr, "fastpath\n");
            }
            bcopy(video_recv->p_data, g_framebufferBase, (video_recv->xres * video_recv->yres * 4));
        } else {
            if (enable_debugging) {
                fprintf(stderr, "slowpath (%f / %f)\n", g_xScaleFactor, g_yScaleFactor);
            }
            uint32_t *inBuf = (uint32_t *)video_recv->p_data;
            uint16_t *outBuf16 = (uint16_t *)g_framebufferBase;
            uint32_t *outBuf32 = (uint32_t *)g_framebufferBase;
            for (int y = 0; y < video_recv->yres; y++) {
                int minRow = scaledRow(y);
                int maxRow = scaledRow(y+1) - 1;
                for (int x = 0; x < video_recv->xres; x++) {
                    uint32_t *inPos = &inBuf[(y * video_recv->xres) + x];
                    for (int outX = scaledColumn(x); outX < scaledColumn(x+1); outX++) {
                        if (monitor_bytes_per_pixel == 4) {
                            uint32_t *outPos32 = &outBuf32[(minRow * g_framebufferXRes) + outX];
                            *outPos32 = *inPos;
                        } else {
                            uint16_t *outPos16 = &outBuf16[(minRow * g_framebufferXRes) + outX];
                            *outPos16 = convert_sample_to_16bpp(*inPos);
                        }
                    }
                }
                for (int row = minRow + 1; row <= maxRow; row++) {
                    if (monitor_bytes_per_pixel == 4) {
                        bcopy(&outBuf32[(minRow * g_framebufferXRes)],
                              &outBuf32[(row * g_framebufferXRes)], g_framebufferXRes * monitor_bytes_per_pixel);
                    } else {
                        bcopy(&outBuf16[(minRow * g_framebufferXRes)],
                              &outBuf16[(row * g_framebufferXRes)], g_framebufferXRes * monitor_bytes_per_pixel);
                    }
                }
            }
        }
        // memset(g_framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));
        // memset(g_framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));

        return true;
    }

    void cleanupFrameBuffer(void) {
        if (ioctl(g_framebufferFileHandle, FBIOPUT_VSCREENINFO, &g_initialFramebufferConfiguration) == -1) {
            fprintf(stderr, "Ioctl FBIOPUT_VSCREENINFO error.\n");
        }
        if (ioctl(g_framebufferFileHandle, FBIOGET_FSCREENINFO, &g_framebufferFixedConfiguration) == -1) {
            fprintf(stderr, "Ioctl FBIOGET_FSCREENINFO.\n");
        }
        munmap(g_framebufferMemory, g_framebufferFixedConfiguration.smem_len);
        close(g_framebufferFileHandle);
    }

#else  // ! __linux__

    BOOL CGImageWriteToFile(CGImageRef image, NSString *path) {
        CFURLRef url = (__bridge CFURLRef)[NSURL fileURLWithPath:path];
        CGImageDestinationRef destination = CGImageDestinationCreateWithURL(url, kUTTypePNG, 1, NULL);
        if (!destination) {
            NSLog(@"Failed to create CGImageDestination for %@", path);
            return NO;
        }

        CGImageDestinationAddImage(destination, image, nil);

        if (!CGImageDestinationFinalize(destination)) {
            NSLog(@"Failed to write image to %@", path);
            CFRelease(destination);
            return NO;
        }

        CFRelease(destination);
        return YES;
    }

    bool drawFrame(NDIlib_video_frame_v2_t *video_recv) {
        if (!configureScreen(video_recv)) {
            return false;
        }
        dispatch_sync(dispatch_get_main_queue(), ^{
            CGContextRef bitmapBuffer = CGBitmapContextCreateWithData(video_recv->p_data, video_recv->xres, video_recv->yres,
                                                                      8, (video_recv->xres * 4), CGColorSpaceCreateDeviceRGB(),
                                                                      kCGImageAlphaNoneSkipFirst | kCGBitmapByteOrder32Little,
                                                                      NULL, NULL);
            CGImageRef imageRef = CGBitmapContextCreateImage(bitmapBuffer);
            NSImage *image = [[NSImage alloc] initWithCGImage:imageRef size:CGSizeMake(video_recv->xres, video_recv->yres)];
            CFRelease(imageRef);
            g_mainImageView.image = image;
        });
        return true;
    }

#endif

void sendPTZUpdates(NDIlib_recv_instance_t pNDI_recv) {
    pthread_mutex_lock(&g_motionMutex);
    motionData_t copyOfMotionData = g_motionData;
    pthread_mutex_unlock(&g_motionMutex);

    // We want to send a "set position X" or "retrieve position X" message only once.  To do this, we
    // keep track of the last set/retrieve command sent, and if the value hasn't changed, we zero
    // the value that we send to the rest of the app.
    static motionData_t lastMotionData = { 0.0, 0.0, 0.0, 0, 0 };

    if (enable_verbose_debugging && copyOfMotionData.zoomPosition != 0) {
        fprintf(stderr, "zSpeed: %f\n", copyOfMotionData.zoomPosition);
    }
    NDIlib_recv_ptz_zoom_speed(pNDI_recv, -copyOfMotionData.zoomPosition);
    NDIlib_recv_ptz_pan_tilt_speed(pNDI_recv, copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
    if (enable_verbose_debugging) {
        if (copyOfMotionData.xAxisPosition != 0 || copyOfMotionData.yAxisPosition != 0) {
            fprintf(stderr, "xSpeed: %f, ySpeed; %f\n", copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
        }
    }
    if (copyOfMotionData.retrievePositionNumber > 0 &&
        copyOfMotionData.retrievePositionNumber != lastMotionData.retrievePositionNumber) {
        fprintf(stderr, "Retrieving position %d\n", copyOfMotionData.retrievePositionNumber);
        NDIlib_recv_ptz_recall_preset(pNDI_recv, copyOfMotionData.retrievePositionNumber, 1.0);  // As fast as possible.
    } else if (copyOfMotionData.storePositionNumber > 0 &&
               copyOfMotionData.storePositionNumber != lastMotionData.storePositionNumber) {
        fprintf(stderr, "Storing position %d\n", copyOfMotionData.storePositionNumber);
        NDIlib_recv_ptz_store_preset(pNDI_recv, copyOfMotionData.storePositionNumber);
    }
    lastMotionData = copyOfMotionData;
}

#ifndef DEMO_MODE
/*
 * Axis (analog) read code.
 *
 *    On Mac:
 *        Returns the floating-point value stored in /var/tmp/axis.%d, else zero.
 *        All values should be floating point values in the range -1 to 1.
 *
 *    On Linux:
 *        Queries a Pimoroni PIM517 I/O Expander.  The button to pin mapping is
 *        provided by the pinNumberForButton function (currently ten greater than
 *        the button number).
 *
 *        All values are converted from the range provided by that hardware into
 *        floating point values in the range -1 to 1.
 *
 *    Values:
 *        Per the NDI spec:
 *
 *            - For the X axis, positive values pan the camera left.
 *            - For the Y axis, positive values tilt the camera upwards.
 *            - For the zoom "axis", positive values (clockwise rotation,
 *              typically) zoom the camera in (closer).
 *
 *        Unless you have a good reason to do otherwise, your hardware should
 *        be built to generate positive and negative values accordingly.
 */
float readAxisPosition(int axis) {
    #ifdef __linux__
	if (!io_expander) return 0;
        int pin = pinNumberForAxis(axis);
        int rawValue = input(io_expander, pin, 0.001) - 2048;
        if (axis == kPTZAxisZoom && abs(rawValue) < 100) {
            rawValue = 0;  // Minimum motion threshold.
        } else if (abs(rawValue) < 10) {
            rawValue = 0;  // Minimum motion threshold.
        }
        float value = rawValue / 2048.0;
        if (value < 0) {
            value = value * -value;  // Logarithmic curve.
        } else {
            value = value * value;  // Logarithmic curve.
        }

        if (enable_verbose_debugging) {
            fprintf(stderr, "axis %d: raw: %d scaled: %f\n", axis, rawValue, value);
        }
        return value;
    #else  // ! __linux__
        char *filename;
        asprintf(&filename, "/var/tmp/axis.%d", axis);
        FILE *fp = fopen(filename, "r");
        free(filename);
        float value = 0.0;
        if (fp) {
            fscanf(fp, "%f\n", &value);
            fclose(fp);
            if (enable_verbose_debugging) {
                fprintf(stderr, "Returning %f for axis %d\n", value, axis);
            }
            return value;
        }
        if (enable_verbose_debugging) {
            fprintf(stderr, "Returning 0.0 (default) for axis %d\n", axis);
        }
        return 0.0;
    #endif
}

/*
 * Button read code.
 *
 *    On Mac:
 *        Returns true if a file exists called /var/tmp/button.%d, else false.
 *
 *    On Linux:
 *        Queries a Pimoroni PIM517 I/O Expander.  The button to pin mapping is
 *        provided by the pinNumberForButton function (currently one greater than
 *        the button number).
 *
 *        The code expects the switch to be open by default, and connected from
 *        the pin to ground, i.e. pressing the button pulls the pin low.  Because
 *        this approach takes advantage of built-in pull-down resistors, it
 *        greatly decreases the risk of a wiring mistake causing you to draw
 *        too much current and crashing or damaging your Raspberry Pi.
 */
bool readButton(int buttonNumber) {
    #ifdef __linux__
	if (!io_expander) return 0;
        int pin = pinNumberForButton(buttonNumber);
        int rawValue = input(io_expander, pin, 0.001);
        bool value = (rawValue == LOW);  // If logic low (grounded), return true.

        if (enable_verbose_debugging) {
            fprintf(stderr, "button %d: raw: %d scaled: %s\n", buttonNumber, rawValue, value ? "true" : "false");
        }
        return value;
    #else  // ! __linux__
        char *filename;
        asprintf(&filename, "/var/tmp/button.%d", buttonNumber);
        FILE *fp = fopen(filename, "r");
        free(filename);
        if (fp) {
            fclose(fp);
            if (enable_verbose_debugging) {
                fprintf(stderr, "Returning true for button %d\n", buttonNumber);
            }
            return true;
        }
        if (enable_verbose_debugging) {
            fprintf(stderr, "Returning false for button %d\n", buttonNumber);
        }
        return false;
    #endif  // __linux__
}

#ifdef __linux__
int pinNumberForAxis(int axis) {
    return axis + 10;
}

int pinNumberForButton(int button) {
    return button + 1;
}
#endif  // __linux__

/*
 * Updates the PTZ values (global variable) from a background thread.  This approach
 * avoids any possibility of a stall while reading the values causing the video
 * playback to malfunction (or worse).  This code uses locks to ensure that it
 * updates the entire set of X/Y/Zoom/button values atomically.
 */
void updatePTZValues() {
    motionData_t newMotionData;

#ifdef DEBUG_HACK
    int readValues[16];

    for (int i = 1; i <= 14; i++) {
        int rawValue = input(io_expander, i, 0.001);
        readValues[i] = rawValue - 2048;
    }

    fprintf(stderr, "    1     2     3     4     5     6     7     8     9    10    11    12    13    14\n");
    fprintf(stderr, "%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n",
        readValues[1], readValues[2], readValues[3], readValues[4], readValues[5],
        readValues[6], readValues[7], readValues[8], readValues[9], readValues[10], readValues[11],
        readValues[12], readValues[13], readValues[14]);

    return;
#endif

    // Update the analog axis values.
    newMotionData.xAxisPosition = readAxisPosition(kPTZAxisX);
    newMotionData.yAxisPosition = readAxisPosition(kPTZAxisY);
    newMotionData.zoomPosition = readAxisPosition(kPTZAxisZoom);

    // Determine whether the set button is down.
    bool isSetButtonDown = readButton(BUTTON_SET);

    // Print a debug message when the user presses or releases the set button,
    // but only once per transition.
    static bool showedInitialState = false;
    static bool lastSetButtonDown = false;
    if (!showedInitialState || (isSetButtonDown != lastSetButtonDown)) {
        fprintf(stderr, "Set button %s\n", isSetButtonDown ? "DOWN" : "UP");
        showedInitialState = true;
    }
    lastSetButtonDown = isSetButtonDown;

    /*
     * Compute the number of the position to store or retrieve.
     *
     * If the set button is down and the user presses a numbered button,
     * storePositionNumber contains that button's number.
     *
     * If the set button is *not* down and the user presses a numbered button,
     * retrievePositionNumber contains that button's number.
     *
     * Otherwise, both values are zero (0).
     */
    newMotionData.storePositionNumber = 0;
    newMotionData.retrievePositionNumber = 0;

    /*
     * NOTE: Button 0 is BUTTON_SET.  Don't query its status here, because you don't
     * reposition the camera when the user presses that button (and because it gets
     * queried above).
     */
    for (int i = 1; i <= MAX_BUTTONS; i++) {
        if (readButton(i)) {
            if (isSetButtonDown) {
                newMotionData.storePositionNumber = i;
            } else {
                newMotionData.retrievePositionNumber = i;
            }
        }
    }

    setMotionData(newMotionData);
}
#endif

void setMotionData(motionData_t newMotionData) {
    /*
     * Lock the motion data mutex and update the motion data so that the NDI code
     * running on the main thread can send it to the camera.
     */
    pthread_mutex_lock(&g_motionMutex);
    g_motionData = newMotionData;
    pthread_mutex_unlock(&g_motionMutex);
}

// Run this computation 200 times per second.  That's more than enough to update
// the state every frame (and then some), and probably enough to update it for
// every audio frame, give or take.  But by only doing this periodically, we
// limit the amount of CPU overhead, leaving more cycles to do the actual
// H.264 or H.265 decoding.
void *runPTZThread(void *argIgnored) {
    while (true) {
#ifdef DEMO_MODE
        demoPTZValues();
#else
        updatePTZValues();
        usleep(5000);
#endif
    }
}

#ifdef DEMO_MODE
void demoPTZValues(void) {
    motionData_t motionData;
    bzero(&motionData, sizeof(motionData));

    // Move the camera for one second at a time.
    motionData.xAxisPosition = 1.0; setMotionData(motionData); usleep(1000000);
    motionData.xAxisPosition = 0.0; setMotionData(motionData); usleep(1000000);
    motionData.yAxisPosition = 1.0; setMotionData(motionData); usleep(1000000);
    motionData.yAxisPosition = 0.0; setMotionData(motionData); usleep(1000000);
    motionData.zoomPosition = 1.0; setMotionData(motionData); usleep(1000000);
    motionData.zoomPosition = 0.0; setMotionData(motionData); usleep(1000000);

    // Store in position 1.
    motionData.storePositionNumber = 1; setMotionData(motionData); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); usleep(1000000);

    motionData.xAxisPosition = -1.0; setMotionData(motionData); usleep(1000000);
    motionData.xAxisPosition = 0.0; setMotionData(motionData); usleep(1000000);
    motionData.yAxisPosition = -1.0; setMotionData(motionData); usleep(1000000);
    motionData.yAxisPosition = 0.0; setMotionData(motionData); usleep(1000000);
    motionData.zoomPosition = -1.0; setMotionData(motionData); usleep(1000000);
    motionData.zoomPosition = 0.0; setMotionData(motionData); usleep(1000000);

    // Store in position 2.
    motionData.storePositionNumber = 2; setMotionData(motionData); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); usleep(1000000);

    motionData.retrievePositionNumber = 1; setMotionData(motionData); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); usleep(1000000);

    motionData.retrievePositionNumber = 2; setMotionData(motionData); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); usleep(1000000);
}
#endif
