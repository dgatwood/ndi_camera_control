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

// #define USE_PREVIEW_RESOLUTION

#ifdef __linux__
    #include <linux/kd.h>
    #include <linux/vt.h>
    #include <linux/fb.h>
    #include <linux/input.h>
    #include <sys/mman.h>
    #include <sys/user.h>

    #define I2C_ADDRESS 0x18

#else
    // Mac (partial support for testing)
    #import <AppKit/AppKit.h>
    #import <CoreServices/CoreServices.h>
    #import <ImageIO/ImageIO.h>
#endif

#include "ioexpander.c"

int monitor_bytes_per_pixel = 4;

#pragma mark - Constants and types

#define SLOW_DEBUGGING 0

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
#else
    NSWindow *g_mainWindow = nil;
    NSImageView *g_mainImageView = nil;
#endif

bool g_ptzEnabled = false;
pthread_mutex_t g_motionMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;

motionData_t g_motionData;

static std::atomic<bool> exit_loop(false);
static void sigint_handler(int)
{    exit_loop = true;
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv);
bool drawFrame(NDIlib_video_frame_v2_t *video_recv);
void *runPTZThread(void *argIgnored);
void sendPTZUpdates(NDIlib_recv_instance_t pNDI_recv);
static int xioctl(int fd, int request, void *arg);

#ifdef __linux__
int pinNumberForAxis(int axis);
int pinNumberForButton(int button);
#endif

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: camera_control \"stream name\"\n");
        fprintf(stderr, "Known sources:\n");
    }
    char *stream_name = argv[1];

#ifdef __linux__
    io_expander = newIOExpander(I2C_ADDRESS, 0, -1, 0, false);
    for (int pin = 0; pin < 2; pin++) {
        ioe_set_mode(io_expander, pinNumberForAxis(pin), PIN_MODE_ADC, false, false);
    }
    // Button 0 is BUTTON_SET.  Configure it like the other buttons.
    for (int button = 0; button <= MAX_BUTTONS; button++) {
        ioe_set_mode(io_expander, pinNumberForButton(button), PIN_MODE_PU, false, false);
    }
#endif

    pthread_t motionThread;
    pthread_create(&motionThread, NULL, runPTZThread, NULL);

#ifndef __linux__
    dispatch_queue_t queue = dispatch_queue_create("ndi run loop", 0);
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.0 * NSEC_PER_SEC)), queue, ^{
#endif

        std::string ndi_path;

        /* BEGIN CRAP: Everything about the code below is gross, but it's boilerplate code. */
        const char* p_NDI_runtime_folder = ::getenv("NDI_RUNTIME_DIR_V4");
        if (p_NDI_runtime_folder) {
            ndi_path = p_NDI_runtime_folder;
            ndi_path += "/libndi.dylib";
        } else {
#ifdef __linux__
            ndi_path = "/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/libndi.so.4";
#else
            ndi_path = "libndi.4.dylib";
#endif
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
            exit(0);
        }

        // Lets get all of the DLL entry points
        const NDIlib_v3 *p_NDILib = NDIlib_v4_load();

        // We can now run as usual
        if (!p_NDILib->NDIlib_initialize())
        {    // Cannot run NDI. Most likely because the CPU is not sufficient (see SDK documentation).
            // you can check this directly with a call to NDIlib_is_supported_CPU()
            printf("Cannot run NDI.");
            exit(0);
        }

        /* END CRAP: Everything about the code above is gross, but it's boilerplate code. */

        // Catch interrupt so that we can shut down gracefully
        signal(SIGINT, sigint_handler);

        // We first need to look for a source on the network
        const NDIlib_find_create_t NDI_find_create_desc = { true, NULL };

        // Create a finder
        NDIlib_find_instance_t pNDI_find = p_NDILib->NDIlib_find_create_v2(&NDI_find_create_desc);
        if (!pNDI_find) exit(0);

        // We wait until there is at least one source on the network
        uint32_t no_sources = 0;
        const NDIlib_source_t *p_sources = NULL;
        while (!exit_loop && !no_sources)
        {    // Wait until the sources on the network have changed
            p_NDILib->NDIlib_find_wait_for_sources(pNDI_find, 1000);
            p_sources = p_NDILib->NDIlib_find_get_current_sources(pNDI_find, &no_sources);
        }

        // We need at least one source
        if (!p_sources) exit(0);

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

#ifdef USE_PREVIEW_RESOLUTION
        NDIlib_recv_create_v3_t NDI_recv_create_desc = { p_sources[source_number], NDIlib_recv_color_format_BGRX_BGRA, NDIlib_recv_bandwidth_lowest, false, "NDIRec" };
#else
        NDIlib_recv_create_v3_t NDI_recv_create_desc = { p_sources[source_number], NDIlib_recv_color_format_BGRX_BGRA, NDIlib_recv_bandwidth_highest, false, "NDIRec" };
#endif

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
#if SLOW_DEBUGGING
                    fprintf(stderr, "Video frame\n");
#endif
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
                    NDIlib_recv_free_audio_v2(pNDI_recv, &audio_recv);
#endif
                default:
#if SLOW_DEBUGGING
                    fprintf(stderr, "Unknown frame type %d.\n", frameType);
#endif
                    true;
            }
            if (g_ptzEnabled) {
                sendPTZUpdates(pNDI_recv);
            } else {
// #if SLOW_DEBUGGING
                fprintf(stderr, "PTZ Disabled\n");
// #endif
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

#ifndef __linux__
        dispatch_async(dispatch_get_main_queue(), ^{
            CFRunLoopStop(CFRunLoopGetMain());
        });
    });
    NSApplicationLoad();
    CFRunLoopRun();
#endif
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
    // g_framebufferMemory = (unsigned char *)mmap(NULL, g_framebufferFixedConfiguration.smem_len + framebufferMemoryOffset, PROT_READ | PROT_WRITE, MAP_SHARED, g_framebufferFileHandle, 0);
    g_framebufferMemory = (unsigned char *)mmap(NULL, g_framebufferFixedConfiguration.smem_len + framebufferMemoryOffset, PROT_READ | PROT_WRITE, MAP_SHARED, g_framebufferFileHandle, 0);
    if ((long)g_framebufferMemory == -1L) {
        perror("cameracontroller: mmap");
        goto fail;
    }

#if 0
    // This is for doing double buffering, but RPi apparently doesn't support that (or vsync in general).
    // move viewport to upper left corner
    if (g_initialFramebufferConfiguration.xoffset != 0 || g_initialFramebufferConfiguration.yoffset != 0) {
        fprintf(stderr, "Shifting framebuffer offset from (%d, %d) to (0, 0)\n", g_initialFramebufferConfiguration.xoffset, g_initialFramebufferConfiguration.yoffset);
        g_initialFramebufferConfiguration.xoffset = 0;
        g_initialFramebufferConfiguration.yoffset = 0;
        if (ioctl(g_framebufferFileHandle, FBIOPAN_DISPLAY, &g_initialFramebufferConfiguration) == -1) {
                perror("cameracontroller: FBIOPAN_DISPLAY");
                munmap(g_framebufferMemory, g_framebufferFixedConfiguration.smem_len);
                goto fail;
        }
    }
#endif

    g_framebufferActiveConfiguration = g_initialFramebufferConfiguration;
    g_framebufferActiveConfiguration.xoffset = 0;
    g_framebufferActiveConfiguration.yoffset = 0;
    // g_framebufferActiveConfiguration.xoffset = 0;
    // g_framebufferActiveConfiguration.yoffset = g_initialFramebufferConfiguration.yres - 1;
    // g_framebufferActiveConfiguration.xres = video_recv->xres;
    // g_framebufferActiveConfiguration.yres = video_recv->yres;

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

#else
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
        // uint32_t scaledX = x / g_xScaleFactor;
        // uint32_t scaledY = y / g_yScaleFactor;
        // return scaledX + (scaledY * g_NDIXRes);
        return (y * g_NDIXRes) + x;
    }

    bool drawFrame(NDIlib_video_frame_v2_t *video_recv) {
        int zero = 0;
        ioctl(g_framebufferFileHandle, FBIO_WAITFORVSYNC, &zero);  // Should work, but doesn't.

        if (!configureScreen(video_recv)) {
            return false;
        }
        if (g_xScaleFactor == 1.0 && g_yScaleFactor && monitor_bytes_per_pixel == 32) {
            bcopy(video_recv->p_data, g_framebufferBase, (video_recv->xres * video_recv->yres * 4));
        } else {
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
                              &outBuf32[(row * g_framebufferXRes)], g_framebufferXRes * 2);
                    } else {
                        bcopy(&outBuf16[(minRow * g_framebufferXRes)],
                              &outBuf16[(row * g_framebufferXRes)], g_framebufferXRes * 2);
                    }
                }
            }
        }
        // memset(g_framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));
        // memset(g_framebufferBase, 0xffffffff, (video_recv->xres * video_recv->yres * 2));

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
        if (ioctl(g_framebufferFileHandle, FBIOPUT_VSCREENINFO, &g_initialFramebufferConfiguration) == -1) {
            fprintf(stderr, "Ioctl FBIOPUT_VSCREENINFO error.\n");
        }
        if (ioctl(g_framebufferFileHandle, FBIOGET_FSCREENINFO, &g_framebufferFixedConfiguration) == -1) {
            fprintf(stderr, "Ioctl FBIOGET_FSCREENINFO.\n");
        }
        munmap(g_framebufferMemory, g_framebufferFixedConfiguration.smem_len);
        close(g_framebufferFileHandle);
    }

#else

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

    if (copyOfMotionData.zoomPosition != 0) {
        fprintf(stderr, "zSpeed: %f\n", copyOfMotionData.zoomPosition);
    }
    NDIlib_recv_ptz_zoom_speed(pNDI_recv, copyOfMotionData.zoomPosition);
    NDIlib_recv_ptz_pan_tilt_speed(pNDI_recv, copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
#if SLOW_DEBUGGING
    if (copyOfMotionData.xAxisPosition != 0 || copyOfMotionData.yAxisPosition != 0) {
        fprintf(stderr, "xSpeed: %f, ySpeed; %f\n", copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
    }
#endif
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

float readAxisPosition(int axis) {
    // For X axis, left should be positive.
    // For Y axis, up should be positive.
    // For zoom, clockwise (zooming in) should be positive.
    #ifdef __linux__
        int pin = pinNumberForAxis(axis);
        int rawValue = input(io_expander, pin, 0.001);
        float value = rawValue / 2048.0;

#if SLOW_DEBUGGING
        fprintf(stderr, "axis %d: raw: %d scaled: %f\n", axis, rawValue, value);
#endif
        return value;
    #else
        char *filename;
        asprintf(&filename, "/var/tmp/axis.%d", axis);
        FILE *fp = fopen(filename, "r");
        free(filename);
        float value = 0.0;
        if (fp) {
            fscanf(fp, "%f\n", &value);
            fclose(fp);
#if SLOW_DEBUGGING
            fprintf(stderr, "Returning %f for axis %d\n", value, axis);
#endif
            return value;
        }
#if SLOW_DEBUGGING
            fprintf(stderr, "Returning 0.0 (default) for axis %d\n", axis, axis);
#endif
        return 0.0;
    #endif
}

bool readButton(int buttonNumber) {
    #ifdef __linux__
        int pin = pinNumberForButton(buttonNumber);
        int rawValue = input(io_expander, pin, 0.001);
        bool value = (rawValue == LOW);  // If logic low (grounded), return true.

#if SLOW_DEBUGGING
        fprintf(stderr, "button %d: raw: %d scaled: %s\n", buttonNumber, rawValue, value ? "true" : "false");
#endif
        return value;
    #else
        // Return true if a file exists called /var/tmp/button.%d.
        char *filename;
        asprintf(&filename, "/var/tmp/button.%d", buttonNumber);
        FILE *fp = fopen(filename, "r");
        free(filename);
        if (fp) {
            fclose(fp);
#if SLOW_DEBUGGING
            fprintf(stderr, "Returning true for button %d\n", buttonNumber);
#endif
            return true;
        }
#if SLOW_DEBUGGING
        fprintf(stderr, "Returning false for button %d\n", buttonNumber);
#endif
        return false;
    #endif
}

#ifdef __linux__
int pinNumberForAxis(int axis) {
    return axis + 10;
}

int pinNumberForButton(int button) {
    return button + 1;
}
#endif


void updatePTZValues() {
    motionData_t newMotionData;

    newMotionData.xAxisPosition = readAxisPosition(kPTZAxisX);
    newMotionData.yAxisPosition = readAxisPosition(kPTZAxisY);
    newMotionData.zoomPosition = readAxisPosition(kPTZAxisZoom);

    bool isSetButtonDown = readButton(BUTTON_SET);
    static bool showedInitialState = false;
    static bool lastSetButtonDown = false;
    if (!showedInitialState || (isSetButtonDown != lastSetButtonDown)) {
        fprintf(stderr, "Set button %s\n", isSetButtonDown ? "DOWN" : "UP");
        showedInitialState = true;
    }
    lastSetButtonDown = isSetButtonDown;
    newMotionData.storePositionNumber = 0;
    newMotionData.retrievePositionNumber = 0;

    // Button 0 is BUTTON_SET.  Don't query its status.
    for (int i = 1; i <= MAX_BUTTONS; i++) {
        if (readButton(i)) {
            if (isSetButtonDown) {
                newMotionData.storePositionNumber = i;
            } else {
                newMotionData.retrievePositionNumber = i;
            }
        }
    }

    pthread_mutex_lock(&g_motionMutex);
    g_motionData = newMotionData;
    pthread_mutex_unlock(&g_motionMutex);
}

void *runPTZThread(void *argIgnored) {
    while (true) {
        updatePTZValues();
        usleep(5000);
    }
}
