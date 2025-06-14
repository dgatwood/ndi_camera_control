#include <csignal>
#include <cstddef>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <semaphore.h>
#include <string>
#include <thread>

#include <math.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <dlfcn.h>

#include <pthread.h>

#define NSEC_PER_SEC 1000000000 

extern "C" {
#ifdef USE_TFBLIB
#include "tfblib/include/tfblib/tfb_colors.h"
#include "tfblib/include/tfblib/tfblib.h"
#endif
}

#include <Processing.NDI.Lib.h>

#define BOOLSTR(boolval) (boolval ? 'Y' : 'N')

#define PACKET_TIMEOUT 1000  /* 1 msec */
#define VISCA_ACK_TIMEOUT 200000  /* 200 milliseconds */
#define MIN_TALLY_INTERVAL 10000 /* 10 msec */
#define PULSES_PER_BLINK 2

#define USE_VISCA_FOR_EXPOSURE_COMPENSATION

#undef ENABLE_FILES_FOR_BUTTON_TESTING

#ifdef __linux__
#define USE_AVAHI
#ifdef USE_MRAA
#include <mraa.h>

#define MAX_PINS 40

volatile bool soft_pwm_enabled[MAX_PINS + 1];   // 1..40 for pins.
volatile uint8_t soft_pwm_value[MAX_PINS + 1];  // 1..40 for pins.
mraa_gpio_context gpio_contexts[MAX_PINS + 1];  // 1..40 for pins.

int set_PWM_dutycycle(__attribute__ ((unused)) int unused,
                  unsigned gpioPin, unsigned dutyCycle);
int gpio_write(int pi, unsigned gpio, unsigned level);
int set_mode(int pi, unsigned gpio, unsigned mode);

void *runPWMThread(void *argIgnored);

#define PI_OUTPUT 42
#else
#include <pigpiod_if2.h>
#endif
#endif // __linux__

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#ifdef USE_AVAHI
    #include <avahi-client/client.h>
    #include <avahi-client/lookup.h>
    #include <avahi-common/simple-watch.h>
    #include <avahi-common/malloc.h>
    #include <avahi-common/error.h>
#else
    #include <dns_sd.h>
#endif // USE_AVAHI

#undef DEMO_MODE

// This must be set correctly in Linux, because the NDI library is bizarre.
#define NDI_LIBRARY_PATH "/usr/local/NDISDK/lib/arm-rpi3-linux-gnueabihf/libndi.so.4"

#include "LightConfiguration.h"

#ifdef USE_ROCKPI_PINOUTS
#define PIMORONI_I2C_FILENAME "/dev/i2c-7"
#else
#define PIMORONI_I2C_FILENAME "/dev/i2c-1"
#endif

#ifdef __linux__
    #include <linux/kd.h>
    #include <linux/vt.h>
    #include <linux/fb.h>
    #include <linux/input.h>
    #include <sys/mman.h>
    #include <sys/user.h>

    #include "LEDConfiguration.h"

    #define I2C_ADDRESS 0x18

#else  // ! __linux__
    // Mac (partial support for testing).  Reads button values from files:
    //     /var/tmp/axis.0 through 2 for X/Y/Zoom values.
    //     /var/tmp/button.0 (set button) - button down if file exists.
    //     /var/tmp/button.1 through 6 - button down if file exists.

    #import <AppKit/AppKit.h>
    #import <CoreServices/CoreServices.h>
    #import <ImageIO/ImageIO.h>

    #define ENABLE_FILES_FOR_BUTTON_TESTING
#endif  // __linux__

#include "ioexpander.c"

int monitor_bytes_per_pixel = 4;
bool monitor_flipped = false;  // Controlled by the -F flag.
bool force_slow_path = false;  // For debugging.

double recallSpeedNDI(void);
int recallSpeedVISCA(void);

#if __linux__
// Always false in macOS.
bool g_use_on_screen_lights = false;
#endif // __linux__

/*
 * Controlled by the -f (--fast) flag.
 *
 * If true, this tool requests a low-quality stream (typically 720p).
 * If false, it requests a high-quality stream (typically 1080p).
 */
bool use_low_res_preview = false;

/* Enable debugging (controlled by the -d / --debug flag). */
bool enable_debugging = false;

/* Enable PTZ debugging (controlled by the -P / --ptzdebug flag). */
bool enable_ptz_debugging = false;

/* Enable debugging (controlled by the -B / --buttondebug flag). */
bool enable_button_debugging = false;

/* Enable debugging (controlled by the -v / --verbose flag). */
bool enable_verbose_debugging = false;

/* Enable VISCA-over-IP camera control. */
bool enable_visca = false;
bool enable_visca_ptz = false;
bool visca_running = false;
bool use_visca_for_presets = false;

double gCurrentPanPosition = 0;   // Only valid if VISCA is enabled.
double gCurrentTiltPosition = 0;  // Only valid if VISCA is enabled.
double gCurrentZoomPosition = 0;  // Only valid if VISCA is enabled.
int gMaxZoomValue = 8;
int gMaxPanTiltValue = 0;  // Default is normal VISCA commands.

#pragma mark - Constants and types

#define safe_asprintf(a, b...) { int retval = asprintf(a, b); \
                                 assert(retval != -1); }

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

/* LED duty cycle. */
uint8_t base_duty_cycle = 255;

#endif // __linux__

#define MAX_BUTTONS 5  // Theoretically, 9, but I don't want to build that much hardware.  Numbered 1 to 5.
#define BUTTON_SET 0   // If the set button is held down, we store a value for that button instead of retrieving it.

typedef struct {
    float xAxisPosition;
    float yAxisPosition;
    float zoomPosition;
    int storePositionNumber;    // Set button is down along with a number button (sent once/debounced).
    int retrievePositionNumber; // A number button is down by itself (sent once/debounced).
    bool setMode;               // True if pushing a button should set the state rather than retrieving it.
    int light[MAX_BUTTONS + 1]; // The current light state (0 .. MAX_BUTTONS)

    // Debounce support.
    bool setButtonDown;         // True if set button is down.
    bool currentValue[MAX_BUTTONS + 1];  // 0 .. MAX_BUTTONS
    bool previousValue[MAX_BUTTONS + 1];  // 0 .. MAX_BUTTONS
    int debounceCounter[MAX_BUTTONS + 1];  // 0 .. MAX_BUTTONS
} motionData_t;

typedef struct receiver_thread_data {
    const NDIlib_v3 *p_NDILib;
    NDIlib_recv_instance_t pNDI_recv;
    char *stream_name;
    std::atomic<bool> running; //(true);
} *receiver_thread_data_t;

typedef struct receiver_array_item {
    char *name;
    NDIlib_recv_instance_t receiver;
    struct receiver_array_item *next;
    pthread_t receiver_thread;
    receiver_thread_data_t thread_data;
} *receiver_array_item_t;

typedef struct visca_network_packet {
    size_t length;
    uint8_t *data;
    bool isInquiry;  // Used for outgoing requests only.
    uint32_t sequenceNumber;  // Used for outgoing requests only.
    double timestamp;
    struct visca_network_packet *next;
    struct visca_network_packet *prev;
} *visca_network_packet_t;

void *runNetworkThread(void *argIgnored);

typedef bool (^packetResponseHandler)(visca_network_packet_t packet);

// Returns true if the packet was handled, else false.
// typedef bool (*response_handler_t)(visca_network_packet_t packet);

typedef struct response_handler_queue_item {
  packetResponseHandler handlerBlock;
  struct response_handler_queue_item *next;
  struct response_handler_queue_item *prev;
} *response_handler_queue_item_t;

response_handler_queue_item_t g_response_handler_queue;

void add_response_handler(packetResponseHandler handler);
void remove_response_handler_item(response_handler_queue_item_t handler);
void remove_response_handler(packetResponseHandler handler);

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

    int g_pig;

    int g_framebufferXRes = 0, g_framebufferYRes = 0, g_NDIXRes = 0, g_NDIYRes = 0;
    double g_xScaleFactor = 0.0, g_yScaleFactor = 0.0;

#else  // ! __linux__
    NSWindow *g_mainWindow = nil;
    NSImageView *g_mainImageView = nil;
#endif  // __linux__

receiver_array_item_t g_shown_sources = NULL;
receiver_array_item_t g_active_receivers = NULL;

void updateLights(motionData_t *motionData);
float scaleAxisValue(int axis, int rawValue);

double getVISCATimestamp(void);

struct timespec last_frame_time;

bool g_ptzEnabled = false;

bool g_set_auto_exposure = false;

bool g_set_exposure_compensation = false;
int8_t g_exposure_compensation = 0;

bool g_set_manual_iris = false;
int8_t g_manual_iris = 0;

bool g_set_manual_gain = false;
int8_t g_manual_gain = 0;

bool g_set_manual_shutter = false;
int8_t g_manual_shutter = 0;

bool g_camera_active = false;
bool g_camera_preview = false;
bool g_camera_malfunctioning = false;

static bool g_fudge_pan = false;
static bool g_fudge_tilt = false;
static bool g_fudge_zoom = false;

static int g_last_zoom_level = 0;
static int g_last_pan_level = 0;
static int g_last_tilt_level = 0;
static bool g_is_loading_preset = false;

static struct sockaddr *g_visca_sockaddr;

typedef struct visca_network_queue {
  visca_network_packet_t head;
  visca_network_packet_t tail;
} *visca_network_queue_t;

static struct visca_network_queue _g_visca_outdata;
static struct visca_network_queue _g_visca_indata;
static visca_network_queue_t g_visca_outdata = &_g_visca_outdata;
static visca_network_queue_t g_visca_indata = &_g_visca_indata;

void push_request(uint8_t *request, size_t length, bool isInquiry, uint32_t sequenceNumber);
void push_response(uint8_t *response, size_t length);
void push_packet(uint8_t *data, size_t length, visca_network_queue_t queue, bool isInquiry, uint32_t sequenceNumber);
int ackType(visca_network_packet_t packet);
visca_network_packet_t deletePacketFromQueue(visca_network_packet_t packet, visca_network_queue_t queue);


// Specs are for Marshall cameras.  Other cameras may differ.
// Shutter Speed | 60/30fps | 50/25fps
// --------------|----------|---------
// 0             | 1/10000  | 1/10000
// 1             | 1/5000   | 1/5000
// 2             | 1/3000   | 1/3000
// 3             | 1/2500   | 1/2500
// 4             | 1/2000   | 1/1750
// 5             | 1/1500   | 1/1250
// 6             | 1/1000   | 1/1000
// 7             | 1/725    | 1/600
// 8             | 1/500    | 1/425
// 9             | 1/350    | 1/300
// 10            | 1/250    | 1/215
// 11            | 1/180    | 1/150
// 12            | 1/120    | 1/120
// 13            | 1/100    | 1/100
// 14            | 1/90     | 1/75
// 15            | 1/60     | 1/50
// 16            | 1/30     | 1/25
// 17            | 1/15     | 1/12
// 18            | 1/8      | 1/6
// 19            | 1/4      | 1/3
// 20            | 1/2      | 1/2
// 21            | 1/1      | 1/1


// Specs are for Marshall cameras.  Other cameras may differ (or even be backwards).
// Gain | Value
// -----|-------
// 15   | +45 dB
// 14   | +42 dB
// 13   | +39 dB
// 12   | +36 dB
// 11   | +33 dB
// 10   | +30 dB
// 9    | +27 dB
// 8    | +24 dB
// 7    | +21 dB
// 6    | +18 dB
// 5    | +15 dB
// 4    | +12 dB
// 3    | +9 dB
// 2    | +6 dB
// 1    | +3 dB
// 0    | 0 dB


// Specs are for Marshall cameras.  Other cameras may differ.
// Iris | Value
// -----|------
// 15   | Close
// 14   | F1.6
// 13   | F2
// 12   | F2.2
// 11   | F2.7
// 10   | F3.2
// 9    | F3.8
// 8    | F4.5
// 7    | F5.4
// 6    | F6.3
// 5    | F7.8
// 4    | F9
// 3    | F11
// 2    | F13
// 1    | F16
// 0    | F18

struct in_addr g_visca_custom_ip;
bool visca_use_custom_ip = false;
int g_visca_sock = -1;
int g_visca_port = 0;
struct sockaddr_in g_visca_addr;
bool g_visca_use_udp = false;

#if __linux__
    pthread_mutex_t g_networkMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;
    pthread_mutex_t g_motionMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;
    pthread_mutex_t g_avahiMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;
#else
    pthread_mutex_t g_networkMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER;
    pthread_mutex_t g_motionMutex = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER;
#endif

motionData_t g_motionData;

#ifdef USE_AVAHI
    AvahiSimplePoll *g_avahi_simple_poll = NULL;
    AvahiClient *g_avahi_client = NULL;
    AvahiServiceBrowser *g_avahi_service_browser = NULL;
#else
    DNSServiceRef g_browseRef = NULL, g_resolveRef = NULL, g_lookupRef = NULL;
#endif

static std::atomic<bool> exit_app(false);
static void sigint_handler(int)
{    exit_app = true;
}

void runUnitTests(void);
void runLightTests(void);
bool configureScreen(NDIlib_video_frame_v2_t *video_recv);
bool drawFrame(NDIlib_video_frame_v2_t *video_recv);
void *runPTZThread(void *argIgnored);
void sendPTZUpdates(NDIlib_recv_instance_t pNDI_recv);
motionData_t getMotionData(void);
void setMotionData(motionData_t newMotionData);
uint32_t find_named_source(const NDIlib_source_t *p_sources,
                           uint32_t no_sources,
                           char *stream_name,
                           bool use_fallback);
bool source_name_compare(const char *name1, const char *name2, bool use_fallback);
bool sourceIsActiveForName(const char *source_name, receiver_array_item_t array);
receiver_array_item_t new_receiver_array_item(void);
void free_receiver_item(receiver_array_item_t receiver_item);
void *runNDIRunLoop(void *receiver_thread_data_ref);
char *fmtbuf(uint8_t *buf, ssize_t size);
void drawOnScreenLights(unsigned char *framebuffer_base, int xres, int yres, int bytes_per_pixel);
void drawOnScreenStats(void);

bool connectVISCA(char *stream_name, const char *context);
void sendVISCALoadPreset(uint8_t presetNumber, int sock);
void sendVISCASavePreset(uint8_t presetNumber, int sock);

#ifdef DEMO_MODE
    void demoPTZValues(void);
#endif

#ifdef __linux__
    bool cc_gpio_write(unsigned gpio, unsigned level);

    int pinNumberForAxis(int axis);
    int pinNumberForButton(int button);

    bool configureGPIO(void);
#endif  // __linux__

// Define to enable a hack that connects to a VISCA device at 127.0.0.1 for
// testing custom VISCA receive code.
#undef PTZ_TESTING

void connectToVISCAPortWithAddress(const struct sockaddr *address);

int main(int argc, char *argv[]) {
    runUnitTests();

#ifdef USE_TFBLIB
   int rc = 0;
   if ((rc = tfb_acquire_fb(0, "/dev/fb0", "/dev/tty6")) != TFB_SUCCESS) {
      fprintf(stderr, "tfb_acquire_fb() failed with error code: %d\n", rc);
      return 1;
   }
#endif

#ifdef USE_MRAA
    pthread_t newSoftPWMThread;
    if (pthread_create(&newSoftPWMThread, NULL, runPWMThread, NULL)) {
        fprintf(stderr, "Could not create PWM thread!\n");
        perror("cameracontroller");
    }
#endif

    // runLightTests();

#ifdef PTZ_TESTING
    enable_visca = true;
    enable_visca_ptz = true;
    // enable_ptz_debugging = true;
    g_visca_use_udp = true;

    g_visca_port = 52381;

    enable_verbose_debugging = false;

    // Create a fake network socket as a stub so that the code doesn't try to
    // connect later.
    struct sockaddr_in address;
    bzero(&address, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = 0;  // Set in connect method.
    inet_aton("127.0.0.1", &address.sin_addr);
    g_visca_sock = directlyConnectToVISCAPortWithAddress((struct sockaddr *)&address);

    fprintf(stderr, "SOCK: %d\n", g_visca_sock);

    visca_running = true;
    pthread_t newMotionThread;
    pthread_create(&newMotionThread, NULL, runPTZThread, NULL);

    while (1) {
        fprintf(stderr, "Active: %s preview: %s malfunctioning: %s\n",
                g_camera_active ? "YES" : " NO",
                g_camera_preview ? "YES" : " NO",
                g_camera_malfunctioning ? "YES" : " NO");
        usleep(1000000);
    }
#endif  // PTZ_TESTING

#ifdef __linux__
    if (!configureGPIO()) {
      fprintf(stderr, "GPIO initialization failed\n");
      return 1;
    }
#endif  // __linux__

    char *stream_name = argv[argc - 1];
    if (argc < 2) {
        fprintf(stderr, "Usage: camera_control \"stream name\"\n");
        fprintf(stderr, "Known sources (polling for 5 seconds):\n");
        stream_name = NULL;
    }
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--debug")) {
            fprintf(stderr, "Enabling debugging (slow).\n");
            enable_debugging = true;
        }
#ifdef __linux__
        if (!strcmp(argv[i], "-D") || !strcmp(argv[i], "--duty_cycle")) {
            fprintf(stderr, "Setting LED brightness.\n");
            if (argc > i + 1) {
                long requested_duty_cycle = atoi(argv[i+1]);
                if (requested_duty_cycle >= 0 &&
                    requested_duty_cycle <= 255) {
                    base_duty_cycle = (uint8_t)requested_duty_cycle;
                } else {
                    fprintf(stderr, "Invalid duty cycle %ld.  (Valid range: 0 to 255)\n", requested_duty_cycle);
                }
                i++;
            }
        }
#endif  // __linux__
#if __linux__
        // Always enabled in macOS.
        if (!strcmp(argv[i], "-O") || !strcmp(argv[i], "--onscreenlights")) {
            fprintf(stderr, "Enabling on-screen (emulated) status lights.\n");
            g_use_on_screen_lights = true;
        }
#endif // __linux__
        if (!strcmp(argv[i], "-B") || !strcmp(argv[i], "--buttondebug")) {
            fprintf(stderr, "Enabling button debugging.\n");
            enable_button_debugging = true;
        }
        if (!strcmp(argv[i], "-P") || !strcmp(argv[i], "--ptzdebug")) {
            fprintf(stderr, "Enabling PTZ debugging.\n");
            enable_ptz_debugging = true;
        }
        if (!strcmp(argv[i], "-R") || !strcmp(argv[i], "--use_visca_for_store_and_recall")) {
            fprintf(stderr, "Enabling VISCA store and recall.\n");
            use_visca_for_presets = true;
        }
        if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose")) {
            fprintf(stderr, "Enabling verbose debugging (slow).\n");
            enable_verbose_debugging = true;
        }
        if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--fast")) {
            fprintf(stderr, "Using low-res mode.\n");
            use_low_res_preview = true;
        }
        if (!strcmp(argv[i], "-F") || !strcmp(argv[i], "--flipped")) {
            fprintf(stderr, "Flipping output\n");
            monitor_flipped = true;
        }
        if (!strcmp(argv[i], "-V") || !strcmp(argv[i], "--enable_visca")) {
            fprintf(stderr, "Enabling VISCA-over-IP control.\n");
            enable_visca = true;
            enable_visca_ptz = true;
        }
        if (!strcmp(argv[i], "-u") || !strcmp(argv[i], "--visca_use_udp")) {
            fprintf(stderr, "Enabling VISCA UDP.\n");
            enable_visca = true;
            g_visca_use_udp = true;
        }
#ifdef USE_VISCA_FOR_EXPOSURE_COMPENSATION
        // NDI only has exposure setting, VISCA only has compensation, iris, gain,
        // and shutter speed.  Ugh.
        if (!strcmp(argv[i], "-e") || !strcmp(argv[i], "--exposure_compensation")) {
            if (argc > i + 1) {
                g_exposure_compensation = atoi(argv[i+1]);
                g_set_exposure_compensation = true;
                i++;
            }
            if (g_exposure_compensation > 5 || g_exposure_compensation < -5) {
                fprintf(stderr, "Invalid exposure compentation %d.  (Valid range: -5 to 5)\n", g_exposure_compensation);
                g_exposure_compensation = 0;
                g_set_exposure_compensation = false;
            } else {
                fprintf(stderr, "Set exposure compentation to %d.\n", g_exposure_compensation);
            }
        }

        if (!strcmp(argv[i], "-a") || !strcmp(argv[i], "--auto_exposure")) {
            g_set_auto_exposure = true;
        }
        if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "--iris")) {
            if (argc > i + 1) {
                g_manual_iris = atoi(argv[i+1]);
                g_set_manual_iris = true;
                i++;
            }
            if (g_manual_iris > 21 || g_manual_iris < 0) {
                fprintf(stderr, "Invalid manual iris %d.  (Valid range: 0 to 21)\n", g_manual_iris);
                g_manual_iris = 0;
                g_set_manual_iris = false;
            } else {
                fprintf(stderr, "Set manual iris to %d.\n", g_manual_iris);
            }
        }

        if (!strcmp(argv[i], "-g") || !strcmp(argv[i], "--gain")) {
            if (argc > i + 1) {
                g_manual_gain = atoi(argv[i+1]);
                g_set_manual_gain = true;
                i++;
            }
            if (g_manual_gain > 15 || g_manual_gain < 0) {
                fprintf(stderr, "Invalid manual gain %d.  (Valid range: 0 to 15)\n", g_manual_gain);
                g_manual_gain = 0;
                g_set_manual_gain = false;
            } else {
                fprintf(stderr, "Set manual iris to %d.\n", g_manual_gain);
            }
        }

        if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "--shutter")) {
            if (argc > i + 1) {
                g_manual_shutter = atoi(argv[i+1]);
                g_set_manual_shutter = true;
                i++;
            }
            if (g_manual_shutter > 21 || g_manual_shutter < 0) {
                fprintf(stderr, "Invalid manual shutter %d.  (Valid range: 0 to 21)\n", g_manual_shutter);
                g_manual_shutter = 0;
                g_set_manual_shutter = false;
            } else {
                fprintf(stderr, "Set manual iris to %d.\n", g_manual_shutter);
            }
        }
#endif  // USE_VISCA_FOR_EXPOSURE_COMPENSATION
        if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--visca_port")) {
            if (argc > i + 1) {
              enable_visca = true;
              g_visca_port = atoi(argv[i+1]);
              i++;
            }
            fprintf(stderr, "Using port %d for VISCA.\n", g_visca_port);
        }
        if (!strcmp(argv[i], "-I") || !strcmp(argv[i], "--visca_ip")) {
            if (argc > i + 1) {
              enable_visca = true;
              if (inet_aton(argv[i+1], &g_visca_custom_ip)) {
                  visca_use_custom_ip = true;
              }
              i++;
            }
            if (visca_use_custom_ip) {
                fprintf(stderr, "Using ip address %s for VISCA.\n", inet_ntoa(g_visca_custom_ip));
            } else {
                fprintf(stderr, "Could not parse custom VISCA IP address.\n");
            }
        }
    }

#ifdef __linux__
#ifndef DEMO_MODE
fprintf(stderr, "Opening I/O Expander at %s\n", PIMORONI_I2C_FILENAME);
    io_expander = newIOExpander(PIMORONI_I2C_FILENAME, I2C_ADDRESS, 5.0, -1, 0, false);
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

    pthread_t networkThread;
    pthread_create(&networkThread, NULL, runNetworkThread, NULL);

#ifndef __linux__
    dispatch_queue_t queue = dispatch_queue_create("ndi run loop", 0);
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.0 * NSEC_PER_SEC)), queue, ^{
#endif  // __linux__

        std::string ndi_path;

        // Try to look up the NDI SDK shared library.
        const char *p_NDI_runtime_folder = ::getenv("NDI_RUNTIME_DIR_V4");
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

        if (!NDIlib_initialize()) {
            printf("The NDI library could not be initialized.");
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
        time_t scanStartTime = time(NULL);

        if (stream_name) {
            fprintf(stderr, "Searching for stream \"%s\"\n", stream_name);
        }
        do {
            if (enable_debugging) fprintf(stderr, "Waiting for source.\n");

            // Wait until the sources on the network have changed
            p_NDILib->NDIlib_find_wait_for_sources(pNDI_find, 1000);
            p_sources = p_NDILib->NDIlib_find_get_current_sources(pNDI_find, &no_sources);

            // If the user provided the name of a stream to display, search for it specifically.
            // Otherwise, just show a list of valid sources and exit.  Either way, iterate
            // through the sources.
            source_number = find_named_source(p_sources, no_sources, stream_name, false);
            if (stream_name != NULL && source_number == -1) {
                source_number = find_named_source(p_sources, no_sources, stream_name, true);
            }

            if (p_sources != NULL && source_number != -1) {
                // See if there is already an active receiver for the exact
                // NDI source name (which includes the IP, typically).
                // If so, don't connect again to the same camera.  If not,
                // reconnect.
                const char *found_source_name = p_sources[source_number].p_ndi_name;
                bool is_active = sourceIsActiveForName(found_source_name, g_active_receivers);

                if (!is_active) {
                    // Create the receiver
                    NDIlib_recv_create_v3_t NDI_recv_create_desc = {
                        p_sources[source_number],
                        NDIlib_recv_color_format_BGRX_BGRA,
                        use_low_res_preview ? NDIlib_recv_bandwidth_lowest : NDIlib_recv_bandwidth_highest,
                        false,
                        "NDIRec"
                    };
                    NDIlib_recv_instance_t pNDI_recv = NDIlib_recv_create_v3(&NDI_recv_create_desc);
                    if (pNDI_recv) {
                        receiver_array_item_t receiver_item = new_receiver_array_item();
                        receiver_item->receiver = pNDI_recv;
                        safe_asprintf(&receiver_item->name, "%s", found_source_name);
                        receiver_item->next = g_active_receivers;
                        receiver_thread_data_t thread_data = (receiver_thread_data_t)malloc(sizeof(*thread_data));
                        thread_data->p_NDILib = p_NDILib;
                        thread_data->pNDI_recv = receiver_item->receiver;
                        safe_asprintf(&thread_data->stream_name, "%s", stream_name);
                        thread_data->running = true;
                        receiver_item->thread_data = thread_data;
                        if (pthread_create(&receiver_item->receiver_thread, NULL, runNDIRunLoop, thread_data) == 0) {
                            g_active_receivers = receiver_item;
                            fprintf(stderr, "Connected.\n");
                            if (g_visca_sock != -1) {
                                close(g_visca_sock);
                                g_visca_sock = -1;
                            }
                            if (enable_visca) {
                                visca_running = connectVISCA(stream_name, "1");
                            }
                        } else {
                            free_receiver_item(receiver_item);
                        }
                    }
                }
            }
            receiver_array_item_t receiver_item = g_active_receivers;
            while (receiver_item != NULL) {
                receiver_array_item_t next_receiver = receiver_item->next;
                if (!receiver_item->thread_data->running) {
                    if (receiver_item->receiver != nullptr) {
                        // Clean up the NDI receiver (stops packet transmission).

                        printf("Closing the connection.\n");
                        p_NDILib->NDIlib_recv_connect(receiver_item->receiver, NULL);

                        printf("Destroying the NDI receiver.\n");
                        p_NDILib->NDIlib_recv_destroy(receiver_item->receiver);
                        printf("The NDI receiver has been destroyed.\n");
                    }
                    if (g_active_receivers == receiver_item) {
                        g_active_receivers = next_receiver;
                    } else {
                        receiver_array_item_t previous_receiver_item = g_active_receivers;
                        while (previous_receiver_item != NULL && previous_receiver_item->next != receiver_item) {
                            previous_receiver_item = previous_receiver_item->next;
                        }
                        if (previous_receiver_item->next == receiver_item) {
                            previous_receiver_item->next = next_receiver;
                        }
                    }
                    free_receiver_item(receiver_item);
                }
                receiver_item = next_receiver;
            }
        } while (!exit_app && (stream_name != NULL || (time(NULL) < scanStartTime + 5)));

        exit_app = true;
        for (receiver_array_item_t receiver_item = g_active_receivers;
             receiver_item != NULL; receiver_item = receiver_item->next) {
            pthread_join(receiver_item->receiver_thread, NULL);
        }

        // Destroy the NDI finder. We needed to have access to the pointers to p_sources[0]
        p_NDILib->NDIlib_find_destroy(pNDI_find);

        // NDIlib_tally_t tallySettings;
        // tallySettings.on_program = false;
        // tallySettings.on_preview = false;
        // NDIlib_recv_set_tally(pNDI_recv, &tallySettings);

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

receiver_array_item_t new_receiver_array_item(void) {
    receiver_array_item_t receiver_item = (receiver_array_item_t)malloc(sizeof(*receiver_item));
    bzero(receiver_item, sizeof(*receiver_item));
    return receiver_item;
}

#pragma mark - Video rendering

void *runNDIRunLoop(void *receiver_thread_data_ref) {
    receiver_thread_data_t thread_data = (receiver_thread_data_t)receiver_thread_data_ref;
    const NDIlib_v3 *p_NDILib = thread_data->p_NDILib;
    NDIlib_recv_instance_t pNDI_recv = thread_data->pNDI_recv;
    char *stream_name = stream_name;

    bool exit_loop = false;
    while(!exit_loop && !exit_app) {
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
            case NDIlib_frame_type_error:
                exit_loop = true;
                break;
            case NDIlib_frame_type_video:
                clock_gettime(CLOCK_REALTIME, &last_frame_time);
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
        if (g_ptzEnabled || enable_visca) {
#ifdef USE_AVAHI
            if (enable_visca && !visca_use_custom_ip) {
                int avahi_error = -1;
                pthread_mutex_lock(&g_avahiMutex);
                if (g_avahi_simple_poll != NULL) {
                    avahi_error = avahi_simple_poll_iterate(g_avahi_simple_poll, 0);
                }
                pthread_mutex_unlock(&g_avahiMutex);
                if (avahi_error != 0) {
                    // Something went horribly wrong.  Just null out the references and start over.
                    g_avahi_simple_poll = NULL;
                    g_avahi_client = NULL;
                    g_avahi_service_browser = NULL;

                    visca_running = connectVISCA(stream_name, "2");
                }
            }
#else
            if (g_browseRef != NULL) {
                DNSServiceProcessResult(g_browseRef);
            }
            if (g_resolveRef != NULL) {
                DNSServiceProcessResult(g_resolveRef);
            }
            if (g_lookupRef != NULL) {
                DNSServiceProcessResult(g_lookupRef);
            }
#endif
            sendPTZUpdates(pNDI_recv);
        } else {
            if (enable_debugging) {
                fprintf(stderr, "PTZ Disabled\n");
            }
        }
    }
    return NULL;
}

bool configureScreen(NDIlib_video_frame_v2_t *video_recv) {
    static bool configured = false;
    if (configured) return true;
    configured = true;

#ifdef __linux__
    // Read the current framebuffer settings so that we can restore them later.
    int framebufferMemoryOffset = 0;
    int zero = 0;
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

    uint16_t convert_sample_to_16bpp(uint32_t sample) {
        // Input:  0xAA RR GG BB          // 32-bit LE integer: 8 bits each for ARGB (alpha high).
        // Output: 0brrrrr gggggg bbbbb;  // 16-bit LE integer: 5 R, 6 G, 5 B (red high).

        uint8_t r = (sample >> 16) & 0xff;
        uint8_t g = (sample >>  8) & 0xff;
        uint8_t b = (sample >>  0) & 0xff;
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (g >> 3);

    }

#ifdef __linux__
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

    bool drawFrame(NDIlib_video_frame_v2_t *video_recv) {
        if (!configureScreen(video_recv)) {
            return false;
        }

        ssize_t screenSize = g_framebufferActiveConfiguration.xres *
                             g_framebufferActiveConfiguration.yres *
                             monitor_bytes_per_pixel;
        static unsigned char *tempBuf = NULL;
        if (tempBuf == NULL) {
            tempBuf = (unsigned char *)mmap(0, screenSize, PROT_READ | PROT_WRITE,
                                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        }
        if (g_xScaleFactor == 1.0 && g_yScaleFactor == 1.0 && monitor_bytes_per_pixel == 4 &&
            !monitor_flipped && !force_slow_path) {

            if (enable_verbose_debugging) {
                fprintf(stderr, "fastpath\n");
            }

            bcopy(video_recv->p_data, tempBuf, screenSize);
            drawOnScreenLights(tempBuf, g_framebufferXRes, g_framebufferYRes, monitor_bytes_per_pixel);
        } else {
            if (enable_verbose_debugging) {
                fprintf(stderr, "slowpath (%f / %f)\n", g_xScaleFactor, g_yScaleFactor);
            }
            uint32_t *inBuf = (uint32_t *)video_recv->p_data;
            uint16_t *outBuf16 = (uint16_t *)tempBuf;
            uint32_t *outBuf32 = (uint32_t *)tempBuf;
            for (int y = 0; y < video_recv->yres; y++) {
                int minRow = scaledRow(y);
                int maxRow = scaledRow(y+1) - 1;
                for (int x = 0; x < video_recv->xres; x++) {
                    int flippedX = monitor_flipped ? video_recv->xres - x - 1 : x;
                    int flippedY = monitor_flipped ? video_recv->yres - y - 1 : y;
                    uint32_t *inPos = &inBuf[(flippedY * video_recv->xres) + flippedX];
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
            drawOnScreenLights(tempBuf, g_framebufferXRes, g_framebufferYRes, monitor_bytes_per_pixel);
        }

        int zero = 0;
        if (ioctl(g_framebufferFileHandle, FBIO_WAITFORVSYNC, &zero) == -1) {
            if (enable_debugging) perror("cameracontroller:  FBIO_WAITFORVSYNC");
        }
        bcopy(tempBuf, g_framebufferBase, screenSize);
        drawOnScreenStats();

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
            ssize_t bufsize = video_recv->xres * video_recv->yres * 4;
            unsigned char *datacopy = (unsigned char *)malloc(bufsize);
            bcopy(video_recv->p_data, datacopy, bufsize);

            drawOnScreenLights(datacopy, video_recv->xres, video_recv->yres, 4);

            CGContextRef bitmapBuffer = CGBitmapContextCreateWithData(datacopy, video_recv->xres, video_recv->yres,
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

enum onScreenColor {
    onScreenLightColorClear = 0,
    onScreenLightColorRed = 1,
    onScreenLightColorGreen = 2,
    onScreenLightColorBlue = 4,
    onScreenLightColorWhite = 7
};

void drawOnScreenStats(void) {

#ifdef USE_TFBLIB
  motionData_t motionData = getMotionData();
  char *string = NULL;
  char *string2 = NULL;
  asprintf(&string, "X: %3.3f XV: %02d Y: %3.3f XV: %02d Z: %3.3f XV: %02d",
    motionData.xAxisPosition, g_last_pan_level, motionData.yAxisPosition,
    g_last_tilt_level, motionData.zoomPosition, g_last_zoom_level);
  asprintf(&string2, "B0: %c B1: %c B2: %c B3: %c B4: %c B5: %c",
    BOOLSTR(motionData.currentValue[0]), BOOLSTR(motionData.currentValue[1]),
    BOOLSTR(motionData.currentValue[2]), BOOLSTR(motionData.currentValue[3]),
    BOOLSTR(motionData.currentValue[4]), BOOLSTR(motionData.currentValue[5]));

  tfb_set_font_by_size(16, 32);
  tfb_draw_string(10, 10, tfb_white, tfb_black, string);
  tfb_draw_string(10, 42, tfb_white, tfb_black, string2);
#endif
}

void drawOnScreenLight(unsigned char *framebuffer_base, int xres, int bytes_per_pixel,
                       int min_x, int max_x, int min_y, int max_y, enum onScreenColor color);

void drawOnScreenLights(unsigned char *framebuffer_base, int xres, int yres, int bytes_per_pixel) {
#if __linux__
    if (g_use_on_screen_lights) {
#endif // __linux__
        motionData_t motionData = getMotionData();

        enum onScreenColor statusColor =
            (g_camera_active && !g_camera_malfunctioning) ? onScreenLightColorRed :
            (g_camera_preview && !g_camera_malfunctioning) ? onScreenLightColorGreen :
            (g_camera_malfunctioning) ? onScreenLightColorBlue :
            onScreenLightColorClear;

        if (statusColor != onScreenLightColorClear) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_STATUS_X_MIN(xres), LIGHT_STATUS_X_MAX(xres),
                              LIGHT_STATUS_Y_MIN(yres), LIGHT_STATUS_Y_MAX(yres), statusColor);
        }
        if (motionData.light[0]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_0_X_MIN(xres), LIGHT_0_X_MAX(xres),
                              LIGHT_0_Y_MIN(yres), LIGHT_0_Y_MAX(yres), onScreenLightColorWhite);
        }
        if (motionData.light[1]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_1_X_MIN(xres), LIGHT_1_X_MAX(xres),
                              LIGHT_1_Y_MIN(yres), LIGHT_1_Y_MAX(yres), onScreenLightColorWhite);
        }
        if (motionData.light[2]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_2_X_MIN(xres), LIGHT_2_X_MAX(xres),
                              LIGHT_2_Y_MIN(yres), LIGHT_2_Y_MAX(yres), onScreenLightColorWhite);
        }
        if (motionData.light[3]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_3_X_MIN(xres), LIGHT_3_X_MAX(xres),
                              LIGHT_3_Y_MIN(yres), LIGHT_3_Y_MAX(yres), onScreenLightColorWhite);
        }
        if (motionData.light[4]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_4_X_MIN(xres), LIGHT_4_X_MAX(xres),
                              LIGHT_4_Y_MIN(yres), LIGHT_4_Y_MAX(yres), onScreenLightColorWhite);
        }
        if (motionData.light[5]) {
            drawOnScreenLight(framebuffer_base, xres, bytes_per_pixel,
                              LIGHT_5_X_MIN(xres), LIGHT_5_X_MAX(xres),
                              LIGHT_5_Y_MIN(yres), LIGHT_5_Y_MAX(yres), onScreenLightColorWhite);
        }
#if __linux__
    }
#endif // __linux__
}

void drawLightPixel(unsigned char *framebuffer_base, ssize_t offset, int color, int bytes_per_pixel);
void drawOnScreenLight(unsigned char *framebuffer_base, int xres, int bytes_per_pixel,
                       int min_x, int max_x, int min_y, int max_y, enum onScreenColor color) {
    ssize_t bytes_per_row = (xres * bytes_per_pixel);
    ssize_t first_row_offset = (bytes_per_row * min_y);
    ssize_t last_row_offset = (bytes_per_row * (max_y + 1));

    for (ssize_t row_start = first_row_offset; row_start < last_row_offset; row_start += bytes_per_row) {
        for (ssize_t rowpos = row_start + (min_x * bytes_per_pixel);
             rowpos < row_start + ((max_x + 1) * bytes_per_pixel);
             rowpos += bytes_per_pixel) {
            drawLightPixel(framebuffer_base, rowpos, color, bytes_per_pixel);
        }
    }
}

void drawLightPixel(unsigned char *framebuffer_base, ssize_t offset, int color, int bytes_per_pixel) {
    bool red = color & onScreenLightColorRed;
    bool green = color & onScreenLightColorGreen;
    bool blue = color & onScreenLightColorBlue;

    uint32_t pixelValue = (red ? 0xFF0000 : 0) | (green ? 0x00FF00 : 0) | (blue ? 0x0000FF : 0);

    if (bytes_per_pixel == 4) {
        uint32_t *pixel = (uint32_t *)(&framebuffer_base[offset]);
        *pixel = pixelValue;
    } else {
        // Assume 2.
        uint16_t *pixel = (uint16_t *)(&framebuffer_base[offset]);
        *pixel = convert_sample_to_16bpp(pixelValue);
    }
}

#pragma mark - VISCA Service Discovery

#ifdef USE_AVAHI

void avahi_client_callback(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void * userdata);
void avahi_resolve_callback(AvahiServiceResolver *r,
                            AVAHI_GCC_UNUSED AvahiIfIndex interface,
                            AVAHI_GCC_UNUSED AvahiProtocol protocol,
                            AvahiResolverEvent event,
                            const char *name,
                            const char *type,
                            const char *domain,
                            const char *host_name,
                            const AvahiAddress *address,
                            uint16_t port,
                            AvahiStringList *txt,
                            AvahiLookupResultFlags flags,
                            AVAHI_GCC_UNUSED void* userdata);
void avahi_browse_callback(AvahiServiceBrowser *browser,
                           AvahiIfIndex interface,
                           AvahiProtocol protocol,
                           AvahiBrowserEvent event,
                           const char *name,
                           const char *type,
                           const char *domain,
                           AVAHI_GCC_UNUSED AvahiLookupResultFlags flags,
                           void *userdata);
void handleDNSResponse(const struct sockaddr *address);

bool connectVISCA(char *stream_name, const char *context) {
    fprintf(stderr, "Called connectVISCA in context %s\n", context);
    if (visca_use_custom_ip) {
        struct sockaddr_in sa;
        bzero(&sa, sizeof(sa));
        sa.sin_family = AF_INET;
        sa.sin_port = 0;
        sa.sin_addr = g_visca_custom_ip;
        handleDNSResponse((sockaddr *)&sa);
        return true;
    }

    pthread_mutex_lock(&g_avahiMutex);

    AvahiSimplePoll *avahi_simple_poll = g_avahi_simple_poll;
    AvahiClient *avahi_client = g_avahi_client;
    AvahiServiceBrowser *avahi_service_browser = g_avahi_service_browser;

    if (!avahi_simple_poll) {
        if (!(avahi_simple_poll = avahi_simple_poll_new())) {
            fprintf(stderr, "Failed to create Avahi run loop.\n");
            pthread_mutex_unlock(&g_avahiMutex);
            return false;
        }
    }
    if (avahi_client == NULL) {
        int error;
        const AvahiPoll *poll = avahi_simple_poll_get(avahi_simple_poll);
        if (!poll) {
            fprintf(stderr, "Avahi poll is broken.\n");
            avahi_simple_poll_free(avahi_simple_poll);
            pthread_mutex_unlock(&g_avahiMutex);
            return false;
        }
        avahi_client = avahi_client_new(poll,
                                          (AvahiClientFlags)0,
                                          avahi_client_callback,
                                          NULL,
                                          &error);
        if (!avahi_client) {
            fprintf(stderr, "Failed to create Avahi client (error %d).\n", error);
            avahi_simple_poll_free(avahi_simple_poll);
            pthread_mutex_unlock(&g_avahiMutex);
            return false;
        }
    }

    if (!(avahi_service_browser = avahi_service_browser_new(avahi_client,
                                                            AVAHI_IF_UNSPEC,
                                                            AVAHI_PROTO_UNSPEC,
                                                            "_ndi._tcp",
                                                            NULL,
                                                            (AvahiLookupFlags)0,
                                                            avahi_browse_callback,
                                                            stream_name))) {
        fprintf(stderr, "Failed to create service browser: %s\n",
                avahi_strerror(avahi_client_errno(avahi_client)));
        avahi_client_free(avahi_client);
        avahi_simple_poll_free(avahi_simple_poll);
        pthread_mutex_unlock(&g_avahiMutex);
        return false;
    }

    g_avahi_simple_poll = avahi_simple_poll;
    g_avahi_client = avahi_client;
    g_avahi_service_browser = avahi_service_browser;

    pthread_mutex_unlock(&g_avahiMutex);
    return true;
}

void avahi_client_callback(AvahiClient *client, AvahiClientState state, AVAHI_GCC_UNUSED void * userdata) {
    if (state == AVAHI_CLIENT_FAILURE) {
        fprintf(stderr, "Server connection failure: %s\n", avahi_strerror(avahi_client_errno(client)));
        avahi_simple_poll_quit(g_avahi_simple_poll);
    }
}

void avahi_browse_callback(AvahiServiceBrowser *browser,
                           AvahiIfIndex interface,
                           AvahiProtocol protocol,
                           AvahiBrowserEvent event,
                           const char *name,
                           const char *type,
                           const char *domain,
                           AVAHI_GCC_UNUSED AvahiLookupResultFlags flags,
                           void *userdata) {
    char *stream_name = (char *)userdata;

    switch (event) {
        case AVAHI_BROWSER_FAILURE:
            fprintf(stderr, "Avahi browser failed: %s\n",
                    avahi_strerror(avahi_client_errno(avahi_service_browser_get_client(browser))));
            avahi_simple_poll_quit(g_avahi_simple_poll);
            // Leak on error.
            g_avahi_simple_poll = NULL;
            g_avahi_client = NULL;
            g_avahi_service_browser = NULL;
            return;
        case AVAHI_BROWSER_NEW:
            if (source_name_compare(name, stream_name, true)) {
                if (!avahi_service_resolver_new(g_avahi_client,
                                                interface,
                                                protocol,
                                                name,
                                                type,
                                                domain,
                                                AVAHI_PROTO_UNSPEC,
                                                (AvahiLookupFlags)0,
                                                avahi_resolve_callback,
                                                g_avahi_client)) {
                    fprintf(stderr, "Failed to resolve service '%s': %s\n",
                            name,
                            avahi_strerror(avahi_client_errno(g_avahi_client)));
                }
            }
            break;
        case AVAHI_BROWSER_REMOVE:
            break;
        case AVAHI_BROWSER_ALL_FOR_NOW:
        case AVAHI_BROWSER_CACHE_EXHAUSTED:
            break;
    }
}

void avahi_resolve_callback(AvahiServiceResolver *resolver,
                            AVAHI_GCC_UNUSED AvahiIfIndex interface,
                            AVAHI_GCC_UNUSED AvahiProtocol protocol,
                            AvahiResolverEvent event,
                            const char *name,
                            const char *type,
                            const char *domain,
                            const char *host_name,
                            const AvahiAddress *address,
                            uint16_t port,
                            AvahiStringList *txt,
                            AvahiLookupResultFlags flags,
                            AVAHI_GCC_UNUSED void* userdata) {

    switch (event) {
        case AVAHI_RESOLVER_FAILURE:
            fprintf(stderr, "Resolver failed: %s\n",
                    avahi_strerror(avahi_client_errno(avahi_service_resolver_get_client(resolver))));
            break;
        case AVAHI_RESOLVER_FOUND:
            if (address->proto == AVAHI_PROTO_INET && g_visca_sock == -1) {
                struct sockaddr_in sa;
                bzero(&sa, sizeof(sa));
                sa.sin_family = AF_INET;
                sa.sin_port = htons(port);
                sa.sin_addr.s_addr = address->data.ipv4.address;
                handleDNSResponse((sockaddr *)&sa);
            }
            if (g_visca_sock == -1) {
                // Keep trying.
                return;
            }
            break;
    }
    avahi_service_resolver_free(resolver);
}

void handleDNSResponse(const struct sockaddr *address) {
    // NewTek's cameras are buggy.  They respond with UDP packets from a different source
    // port than the port we send to, which makes connected UDP sockets impossible.  This
    // sucks from a performance perspective, but we work around it by not connecting the
    // socket.
    connectToVISCAPortWithAddress(address);
}

#else // ! USE_AVAHI

void handleDNSServiceResolveReply(DNSServiceRef sdRef,
                                  DNSServiceFlags flags,
                                  uint32_t interfaceIndex,
                                  DNSServiceErrorType errorCode,
                                  const char *fullname,
                                  const char *hosttarget,
                                  uint16_t port, /* In network byte order */
                                  uint16_t txtLen,
                                  const unsigned char *txtRecord,
                                  void *context);

void handleDNSServiceGetAddrInfoReply(DNSServiceRef sdRef,
                                      DNSServiceFlags flags,
                                      uint32_t interfaceIndex,
                                      DNSServiceErrorType errorCode,
                                      const char *hostname,
                                      const struct sockaddr *address,
                                      uint32_t ttl,
                                      void *context);

void handleDNSServiceBrowseReply(DNSServiceRef sdRef,
                                 DNSServiceFlags flags,
                                 uint32_t interfaceIndex,
                                 DNSServiceErrorType errorCode,
                                 const char *serviceName,
                                 const char *regtype,
                                 const char *replyDomain,
                                 void *context);

bool connectVISCA(char *stream_name, const char *context) {
    if (g_browseRef != NULL) {
        DNSServiceRefDeallocate(g_browseRef);
        g_browseRef = NULL;
    }
    // Start browsing for the service.
    fprintf(stderr, "Starting browser.\n");

    // Try several times, because this API sometimes fails at random with
    // errno = EPERM (operation not permitted).
    for (int i = 0 ; i < 5; i++) {
        DNSServiceErrorType errorCode =
            DNSServiceBrowse(&g_browseRef, 0, 0,
                             "_ndi._tcp",
                             NULL,
                             &handleDNSServiceBrowseReply,
                             (void *)stream_name);
        if (errorCode != kDNSServiceErr_NoError) {
            perror("cameracontroller");
            fprintf(stderr, "Could not start service browser for VISCA (error %d)\n", errorCode);
        } else {
            return true;
        }
    }
    return false;
}

void handleDNSServiceBrowseReply(DNSServiceRef sdRef,
                                 DNSServiceFlags flags,
                                 uint32_t interfaceIndex,
                                 DNSServiceErrorType errorCode,
                                 const char *serviceName,
                                 const char *regtype,
                                 const char *replyDomain,
                                 void *context) {
    if (errorCode) {
        fprintf(stderr, "Service browser for VISCA failed (error %d)\n", errorCode);
        DNSServiceRefDeallocate(sdRef);
        g_browseRef = NULL;
        connectVISCA((char *)context, "3");
        return;
    }
    if (g_resolveRef != NULL) {
        DNSServiceRefDeallocate(g_resolveRef);
        g_resolveRef = NULL;
    }
    char *stream_name = (char *)context;
    fprintf(stderr, "Got service %s\n", serviceName);
    if (source_name_compare(serviceName, stream_name, true)) {
        fprintf(stderr, "MATCH\n");
        if (g_resolveRef != NULL) {
            DNSServiceRefDeallocate(g_browseRef);
            g_resolveRef = NULL;
        }
        fprintf(stderr, "Starting resolver.\n");
        if (DNSServiceResolve(&g_resolveRef, 0, interfaceIndex, serviceName, regtype, replyDomain, &handleDNSServiceResolveReply, context)
                              != kDNSServiceErr_NoError) {
            fprintf(stderr, "Could not start service resolver for VISCA (error %d)\n", errorCode);
            DNSServiceRefDeallocate(sdRef);
            g_browseRef = NULL;
            connectVISCA((char *)context, "4");
            return;
        }
    } else {
        fprintf(stderr, "NOMATCH\n");
    }
    if (flags & kDNSServiceFlagsMoreComing) { return; }  // Keep browsing until we have a full response.
    DNSServiceRefDeallocate(sdRef);
    g_browseRef = NULL;
}

void handleDNSServiceResolveReply(DNSServiceRef sdRef,
                                  DNSServiceFlags flags,
                                  uint32_t interfaceIndex,
                                  DNSServiceErrorType errorCode,
                                  const char *fullname,
                                  const char *hosttarget,
                                  uint16_t port,
                                  uint16_t txtLen,
                                  const unsigned char *txtRecord,
                                  void *context) {
    fprintf(stderr, "VISCA resolved\n");
    if (errorCode) {
        fprintf(stderr, "Service resolver for VISCA failed (error %d)\n", errorCode);
        DNSServiceRefDeallocate(sdRef);
        g_resolveRef = NULL;
        connectVISCA((char *)context, "5");
        return;
    }
    if (g_lookupRef != NULL) {
        DNSServiceRefDeallocate(g_lookupRef);
        g_lookupRef = NULL;
    }
    if (DNSServiceGetAddrInfo(&g_lookupRef, 0, interfaceIndex, kDNSServiceProtocol_IPv4, hosttarget, &handleDNSServiceGetAddrInfoReply, context)
                              != kDNSServiceErr_NoError)
    {
        fprintf(stderr, "Could not start host resolver for VISCA (error %d)\n", errorCode);
        DNSServiceRefDeallocate(sdRef);
        g_resolveRef = NULL;
        connectVISCA((char *)context, "6");
        return;
    }

    if (flags & kDNSServiceFlagsMoreComing) { return; }  // Keep resolving until we have a full response.
    DNSServiceRefDeallocate(sdRef);
    g_resolveRef = NULL;
}

void handleDNSServiceGetAddrInfoReply(DNSServiceRef sdRef,
                                      DNSServiceFlags flags,
                                      uint32_t interfaceIndex,
                                      DNSServiceErrorType errorCode,
                                      const char *hostname,
                                      const struct sockaddr *address,
                                      uint32_t ttl,
                                      void *context) {

    if (errorCode != kDNSServiceErr_NoError) {
        fprintf(stderr, "Service resolver for VISCA failed (error %d)\n", errorCode);
    } else {
        connectToVISCAPortWithAddress(address);
        if (g_visca_sock == -1) {
            fprintf(stderr, "VISCA failed.");
            if (flags & kDNSServiceFlagsMoreComing) { return; }  // Keep browsing until we have a full response.
            DNSServiceRefDeallocate(sdRef);
            g_lookupRef = NULL;
            return;
        }
        fprintf(stderr, "VISCA ready.\n");
    }

    if (flags & kDNSServiceFlagsMoreComing) { return; }  // Keep browsing until we have a full response.
    DNSServiceRefDeallocate(sdRef);
    g_lookupRef = NULL;
}

#endif // USE_AVAHI

#pragma mark - VISCA Core

void connectToVISCAPortWithAddress(const struct sockaddr *address) {
    size_t addr_size = 0;
    if (address->sa_family == AF_INET) {
      addr_size = sizeof(struct sockaddr_in);
    } else if (address->sa_family == AF_INET6) {
      addr_size = sizeof(struct sockaddr_in6);
    } else {
      fprintf(stderr, "Ignoring unknown family %d in connectToVISCAPortWithAddress", address->sa_family);
      return;
    }
    struct sockaddr *copiedAddress = (struct sockaddr *)malloc(addr_size);
    memcpy((void *)copiedAddress, (void *)address, addr_size);

    pthread_mutex_lock(&g_networkMutex);

    struct sockaddr *oldAddress = g_visca_sockaddr;
    g_visca_sockaddr = copiedAddress;
    free(oldAddress);

    close(g_visca_sock);
    g_visca_sock = -1;

    pthread_mutex_unlock(&g_networkMutex);
}

int directlyConnectToVISCAPortWithAddress(const struct sockaddr *address) {
    struct sockaddr_in sa;
    memcpy((void *)&sa, (void *)address, sizeof(struct sockaddr_in));

    if (g_visca_port != 0) {
        fprintf(stderr, "Connecting to custom VISCA port %d\n", g_visca_port);
        sa.sin_port = htons(g_visca_port);
    } else {
        // Marshall uses 52381 UDP; PTZOptics uses 5678 TCP or 1259 UDP.
        sa.sin_port = g_visca_use_udp ? htons(1259) : htons(5678);
    }

    fprintf(stderr, "Connecting to VISCA port %d on %s\n", htons(sa.sin_port), inet_ntoa(sa.sin_addr));

    int sock = g_visca_use_udp ?
        socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) :
        socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1) {
        perror("Socket could not be created.");
        return -1;
    }
    int reuseValue = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuseValue, sizeof(reuseValue));

    struct sockaddr_in sa_recv;
    bzero(&sa_recv, sizeof(sa_recv));
    sa_recv.sin_family = AF_INET;
    sa_recv.sin_addr.s_addr = INADDR_ANY;
    sa_recv.sin_port = htons(g_visca_port + 1);
    if (bind(sock, (sockaddr *)&sa_recv, sizeof(sa_recv)) != 0) {
        perror("Bind failed");
        close(sock);
        return -1;
    }

    if (g_visca_use_udp) {
        g_visca_addr = sa;
    } else {
        if (connect(sock, (struct sockaddr *) &sa, sizeof(sa)) == -1) {
            perror("Connect failed");
            close(sock);
            return -1;
        }
    }
    return sock;
}

void updateVISCAMaxSpeed(int sock);
void updateTallyLightsOverVISCA(int sock);
void updateZoomPositionOverVISCA(int sock);
bool updateZoomPositionOverVISCA(int sock, bool digitalMode);
void sendZoomUpdatesOverVISCA(int sock, motionData_t *motionData);
void sendPanTiltUpdatesOverVISCA(int sock, motionData_t *motionData);
void sendExposureCompensationOverVISCA(int sock);
void sendManualExposureOverVISCA(int sock);

void sendPTZUpdatesOverVISCA(motionData_t *motionData) {
    updateVISCAMaxSpeed(g_visca_sock);
    updateTallyLightsOverVISCA(g_visca_sock);
    updateZoomPositionOverVISCA(g_visca_sock);
    if (enable_visca_ptz) {
        sendZoomUpdatesOverVISCA(g_visca_sock, motionData);
        sendPanTiltUpdatesOverVISCA(g_visca_sock, motionData);
#ifdef USE_VISCA_FOR_EXPOSURE_COMPENSATION
        sendExposureCompensationOverVISCA(g_visca_sock);
        sendManualExposureOverVISCA(g_visca_sock);
#endif
    }
}

int send_visca_packet(int sock, uint8_t *buf, ssize_t bufsize, int timeout_usec, bool isInquiry,
                      uint32_t *outSequenceNumber);
uint8_t *send_visca_inquiry(int sock, uint8_t *buf, ssize_t bufsize, int timeout_usec, ssize_t *responseLength);

// Custom VISCA command, because 8 speeds aren't enough to properly drive Panasonic cameras.  This is a
// nonstandard command specific to the VISCAPTZ project.  On all actual VISCA devices, this will fail,
// hence the large response packet with known values.
void updateVISCAMaxSpeed(int sock) {
    static struct timeval lastCheck = { 0, 0 };
    struct timeval curTime;

    gettimeofday(&curTime, NULL);

    // This should never change, but we need to keep trying until we get a valid result, and it
    // isn't worth stopping the polling after we do.
    if ((curTime.tv_sec - lastCheck.tv_sec) < 5) return;

    lastCheck = curTime;

    uint8_t buf[5] = { 0x81, 0x09, 0x04, 0x07, 0xFF };  // Custom max zoom speed command
    ssize_t responseLength = 0;
    uint8_t *responseBuf = send_visca_inquiry(sock, buf, sizeof(buf), 20000, &responseLength);
    if (responseBuf && responseLength == 13 &&
            responseBuf[1] == 0x50 && responseBuf[2] == 0xde &&
            responseBuf[3] == 0xad && responseBuf[4] == 0xbe &&
            responseBuf[5] == 0xef && responseBuf[6] == 0xfe &&
            responseBuf[7] == 0xed && responseBuf[8] == 0xba &&
            responseBuf[9] == 0xbe && responseBuf[12] == 0xff) {
        gMaxZoomValue = responseBuf[10] << 8 | responseBuf[11];
        // gMaxPanTiltValue = responseBuf[12] << 8 | responseBuf[13];
        fprintf(stderr, "Max zoom value changed to %d\n", gMaxZoomValue);
    } else {
        if (enable_verbose_debugging) {
            // This is a nonstandard command specific to cameras that support a VISCA extension
            // that allows much more precise speed control.  Don't report this as an error,
            // because it will fail on nearly all VISCA devices.
            fprintf(stderr, "Bad response %s for max zoom value (length %" PRId64 ").\n", // ssize_t
                fmtbuf(responseBuf, responseLength), (uint64_t)responseLength);
        }
    }
}

void updatePanTiltPositionOverVISCA(int sock) {
    static struct timeval lastCheck = { 0, 0 };
    struct timeval curTime;

    static double lastPanPosition = 0;
    static double lastTiltPosition = 0;
    static int panMessageFailCount = 0;
    static int tiltMessageFailCount = 0;
    static int panMessageSuccessCount = 0;
    static int tiltMessageSuccessCount = 0;
    static bool isSet = false;

    static int minSuccessToIgnore = 1000000 / MIN_TALLY_INTERVAL;  // One second.
    static int minFailureToIgnore = 50000 / MIN_TALLY_INTERVAL;  // 50 ms.

    gettimeofday(&curTime, NULL);

    uint64_t difference = (curTime.tv_usec - lastCheck.tv_usec) +
        ((curTime.tv_sec != lastCheck.tv_sec) ? 1000000 : 0);

    if (difference < MIN_TALLY_INTERVAL) return;
    lastCheck = curTime;

    uint8_t buf[5] = { 0x81, 0x09, 0x06, 0x12, 0xFF };

    ssize_t responseLength = 0;
    if (enable_verbose_debugging) {
        fprintf(stderr, "Getting pan and tilt position.\n");
    }
    uint8_t *responseBuf = send_visca_inquiry(sock, buf, sizeof(buf), 20000, &responseLength);
    if (responseBuf && responseLength == 11 && responseBuf[1] == 0x50 && responseBuf[3] == 0xff) {
        if (enable_verbose_debugging) {
            fprintf(stderr, "Got pan and tilt position data: %s\n", fmtbuf(responseBuf, responseLength));
        }
        int rawPanValue = ((responseBuf[2] & 0xf) << 12) | ((responseBuf[3] & 0xf) << 8) |
            ((responseBuf[4] & 0xf) << 4) | (responseBuf[5] & 0xf);
        int rawTiltValue = ((responseBuf[6] & 0xf) << 12) | ((responseBuf[7] & 0xf) << 8) |
            ((responseBuf[8] & 0xf) << 4) | (responseBuf[9] & 0xf);
        gCurrentPanPosition = (double)rawPanValue / (double)0xFFFF;
        gCurrentTiltPosition = (double)rawTiltValue / (double)0xFFFF;
    } else {
        fprintf(stderr, "Bad response %s for pan/tilt position value (length %" PRId64 ").\n",
            fmtbuf(responseBuf, responseLength), (uint64_t)responseLength);
    }
    if (!isSet) {
        isSet = true;
    } else { 
        if (g_last_pan_level == 0 && lastPanPosition != gCurrentPanPosition) {
            // If the camera should not be moving, but is, increment the
            // failure count.
            panMessageFailCount++;
            panMessageSuccessCount = 0;
        } else if (g_last_pan_level == 0) {
            // If the camera is not supposed to be moving and is not, increment
            // The success count.  If it gets high enough (no motion in one
            // second), ignore previous failures.
            if (panMessageSuccessCount++ > minSuccessToIgnore) {
                panMessageFailCount = 0;
            }
        } else {
            // If the camera is supposed to be moving, don't check at all.
            panMessageSuccessCount = 0;
            panMessageFailCount = 0;
        }
        if (panMessageFailCount >= minFailureToIgnore) {
            fprintf(stderr, "@@@ WARNING: FUDGING PAN BECAUSE CAMERA DID NOT STOP");
            g_fudge_pan = true;
        }

        if (g_last_tilt_level == 0 && lastTiltPosition != gCurrentTiltPosition) {
            // If the camera should not be moving, but is, increment the
            // failure count.
            tiltMessageFailCount++;
            tiltMessageSuccessCount = 0;
        } else if (g_last_tilt_level == 0) {
            // If the camera is not supposed to be moving and is not, increment
            // The success count.  If it gets high enough (no motion in one
            // second), ignore previous failures.
            if (tiltMessageSuccessCount++ > minSuccessToIgnore) {
                tiltMessageFailCount = 0;
            }
        } else {
            // If the camera is supposed to be moving, don't check at all.
            tiltMessageSuccessCount = 0;
            tiltMessageFailCount = 0;
        }
        if (panMessageFailCount >= minFailureToIgnore) {
            fprintf(stderr, "@@@ WARNING: FUDGING TILT BECAUSE CAMERA DID NOT STOP");
            g_fudge_tilt = true;
        }

        lastPanPosition = gCurrentPanPosition;
        lastTiltPosition = gCurrentTiltPosition;
    }
}

void updateZoomPositionOverVISCA(int sock) {
  static bool useDigitalMode = false;

  if (sock == -1) {
    return;
  }

  if (useDigitalMode) {
    if (!updateZoomPositionOverVISCA(sock, true)) {
      useDigitalMode = false;
      fprintf(stderr, "Camera does not support digital zoom query.  Using optical query.");
      updateZoomPositionOverVISCA(sock, false);
    }
  } else {
    if (!updateZoomPositionOverVISCA(sock, false)) {
      useDigitalMode = true;
      fprintf(stderr, "Camera does not support optical zoom query.  Using digital query.");
      updateZoomPositionOverVISCA(sock, true);
    }
  }
}

bool updateZoomPositionOverVISCA(int sock, bool digitalMode) {
    static struct timeval lastCheck = { 0, 0 };
    struct timeval curTime;

    static double lastZoomPosition = 0;
    static int zoomMessageFailCount = 0;
    static int zoomMessageSuccessCount = 0;
    static bool isSet = false;

    static int minSuccessToIgnore = 1000000 / MIN_TALLY_INTERVAL;  // One second.
    static int minFailureToIgnore = 50000 / MIN_TALLY_INTERVAL;  // 50 ms.


    gettimeofday(&curTime, NULL);

    uint64_t difference = (curTime.tv_usec - lastCheck.tv_usec) +
        ((curTime.tv_sec != lastCheck.tv_sec) ? 1000000 : 0);

    if (difference < MIN_TALLY_INTERVAL) return true;
    lastCheck = curTime;

    uint8_t buf[5] = { 0x81, 0x09, 0x04, 0x47, 0xFF };
    if (digitalMode) {
      buf[3] = 0x46; // { 0x81, 0x09, 0x04, 0x46, 0xFF };
    }

    ssize_t responseLength = 0;
    if (enable_verbose_debugging) {
        fprintf(stderr, "Getting zoom position.\n");
    }
    uint8_t *responseBuf = send_visca_inquiry(sock, buf, sizeof(buf), 20000, &responseLength);
    bool failed = false;
    if (responseBuf && responseLength == 7 && responseBuf[1] == 0x50 && responseBuf[6] == 0xff) {
        if (enable_verbose_debugging) {
            fprintf(stderr, "Got zoom position data: %s\n", fmtbuf(responseBuf, responseLength));
        }
        int rawValue = ((responseBuf[2] & 0xf) << 12) | ((responseBuf[3] & 0xf) << 8) |
            ((responseBuf[4] & 0xf) << 4) | (responseBuf[5] & 0xf);
        gCurrentZoomPosition = (double)rawValue / (double)0xFFFF;
    } else {
        // For some reason, some major brand cameras periodically drop this request (and only
        // this request).  Silence the error, because it is annoying.
        if (enable_verbose_debugging) {
            fprintf(stderr, "Bad response %s for zoom position value (length %" PRId64 ").\n",
                fmtbuf(responseBuf, responseLength), (uint64_t)responseLength);
        }
        if (responseLength > 0) {
            // Return a failure if and only if the response from the camera is garbage.  A
            // zero-length response is probably just a timeout, and we don't want this
            // software to flip-flop back and forth between the two modes indiscriminately.
            failed = true;
        }
    }
#if 0
    fprintf(stderr, "Last zoom speed: %d\n", g_last_zoom_level);
    fprintf(stderr, "Last zoom position: %lf\nCurrent zoom position: %lf\n", lastZoomPosition, gCurrentZoomPosition);
    fprintf(stderr, "zoomMessageFailCount: %d\nzoomMessageSuccessCount: %d\nminSuccessToIgnore: %d\nminFailureToIgnore: %d\n",
            zoomMessageFailCount, zoomMessageSuccessCount, minFailureToIgnore, minFailureToIgnore);
#endif
    if (!isSet) {
        isSet = true;
    } else if (!g_is_loading_preset) { 
        if (g_last_zoom_level == 0 && lastZoomPosition != gCurrentZoomPosition) {
            // If the camera should not be moving, but is, increment the
            // failure count.
            zoomMessageFailCount++;
            zoomMessageSuccessCount = 0;
        } else if (g_last_zoom_level == 0) {
            // If the camera is not supposed to be moving and is not, increment
            // The success count.  If it gets high enough (no motion in one
            // second), ignore previous failures.
            if (zoomMessageSuccessCount++ > minSuccessToIgnore) {
                zoomMessageFailCount = 0;
            }
        } else {
            // If the camera is supposed to be moving, don't check at all.
            zoomMessageSuccessCount = 0;
            zoomMessageFailCount = 0;
        }
        if (zoomMessageFailCount >= minFailureToIgnore) {
            fprintf(stderr, "@@@ WARNING: FUDGING ZOOM BECAUSE CAMERA DID NOT STOP\n");
            g_fudge_zoom = true;
        }

        // fprintf(stderr, "lastZoomPosition: %lf current: %lf\n", lastZoomPosition, gCurrentZoomPosition);
        lastZoomPosition = gCurrentZoomPosition;
    }
    return !failed;
}

void updateTallyLightsOverVISCA(int sock) {
    static struct timeval lastCheck = { 0, 0 };
    struct timeval curTime;

    gettimeofday(&curTime, NULL);

    uint64_t difference = (curTime.tv_usec - lastCheck.tv_usec) +
        ((curTime.tv_sec != lastCheck.tv_sec) ? 1000000 : 0);

    if (difference < MIN_TALLY_INTERVAL) return;
    lastCheck = curTime;

    uint8_t buf[7] = { 0x81, 0x09, 0x7E, 0x01, 0x0A, 0x01, 0xFF };

    // To improve compatibility, if this fails, we should also try:
    // 81 09 7E 01 0A FF (red check) -> 90 50 0p FF where p is 2 (on) or 3 (off)
    // 81 09 7E 04 1A FF (green check) -> 90 50 0p FF where p is 2 (on) or 3 (off)

    ssize_t responseLength = 0;
    uint8_t *responseBuf = send_visca_inquiry(sock, buf, sizeof(buf), 20000, &responseLength);
    if (responseBuf && responseLength == 4 && responseBuf[1] == 0x50 && responseBuf[3] == 0xff) {
        if (enable_verbose_debugging) {
            fprintf(stderr, "Got tally data: %s\n", fmtbuf(responseBuf, responseLength));
        }
        int tallyMode = responseBuf[2];
        if (tallyMode == 0) {
            g_camera_active = false;
            g_camera_preview = false;
        } else if (tallyMode == 5) {
            g_camera_active = true;
            g_camera_preview = false;
        } else if (tallyMode == 6) {
            g_camera_active = false;
            g_camera_preview = true;
        } else {
            fprintf(stderr, "Unknown tally mode %d\n", tallyMode);
        }
    }
}

void sendVISCASetRecallSpeed(uint8_t speed, int sock) {
    // uint8_t buf[7] = { 0x81, 0x01, 0x06, 0x01, speed, 0xFF };
    // send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
}

void sendVISCALoadPreset(uint8_t presetNumber, int sock) {
    int speed = recallSpeedVISCA();
    sendVISCASetRecallSpeed(speed, sock);

    uint8_t buf[7] = { 0x81, 0x01, 0x04, 0x3F, 0x02, presetNumber, 0xFF };
    send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
}

void sendVISCASavePreset(uint8_t presetNumber, int sock) {
    uint8_t buf[7] = { 0x81, 0x01, 0x04, 0x3F, 0x01, presetNumber, 0xFF };
    send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
}

void sendZoomUpdatesOverVISCA(int sock, motionData_t *motionData) {
    if (gMaxZoomValue != 8) {
        // Use a nonstandard VISCA command with a much larger zoom speed range.
        uint8_t buf[8] = { 0x81, 0x01, 0x04, 0x07, 0x00, 0x00, 0x00, 0xFF };
        int level = (int)(motionData->zoomPosition * (gMaxZoomValue + 0.9));

        static int count = 0;

        if (level == g_last_zoom_level) {
            return;
        }
        g_last_zoom_level = level;

        buf[4] = (level < 0) ? 0x2f : 0x3f;
        buf[5] = (abs(level) >> 8) & 0xff;
        buf[6] = abs(level) & 0xff;

        if (enable_ptz_debugging) {
            fprintf(stderr, "zoom speed: %d buf: 0x%02x\n", level, buf[4]);
        }
    
        send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
    } else {
        uint8_t buf[6] = { 0x81, 0x01, 0x04, 0x07, 0x00, 0xFF };
        int level = (int)(motionData->zoomPosition * (8.9));

        // Certain major-brand PTZ cameras have a defect where they sometimes
        // lose VISCA commands and think that they have changed the speed when
        // they actually have not.  If the camera is still moving and should be
        // stopped, send a single nonzero motion command and then immediately
        // send a stop command again to work around the bug.
        if (g_fudge_zoom) {
            g_fudge_zoom = false;
            level = (level <= 0) ? level + 1 : level - 1;
        }

        if (level == g_last_zoom_level) {
            return;
        }
        fprintf(stderr, "Setting g_last_zoom_level to %d\n", level);
        g_last_zoom_level = level;

        if (level != 0) {
            buf[4] = (abs(level) - 1) | (level < 0 ? 0x20 : 0x30);
        }
        if (enable_ptz_debugging) {
            fprintf(stderr, "zoom speed: %d buf: 0x%02x\n", level, buf[4]);
        }

        send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
    }
}

void sendExtendedPanTiltUpdatesOverVISCA(int sock, motionData_t *motionData);
void sendPanTiltUpdatesOverVISCA(int sock, motionData_t *motionData) {
    if (gMaxPanTiltValue != 0) {
      sendExtendedPanTiltUpdatesOverVISCA(sock, motionData);
      return;
    }
    const bool localDebug = false;
    int pan_level = (int)(motionData->xAxisPosition * 24.9);
    int tilt_level = (int)(motionData->yAxisPosition * 23.9);
    // fprintf(stderr, "VISCA MODE: %d, %d\n", pan_level, tilt_level);

    // Certain major-brand PTZ cameras have a defect where they sometimes
    // lose VISCA commands and think that they have changed the speed when
    // they actually have not.  If the camera is still moving and should be
    // stopped, send a single nonzero motion command and then immediately
    // send a stop command again to work around the bug.
    if (g_fudge_pan) {
        g_fudge_pan = false;
        pan_level = (pan_level <= 0) ? pan_level + 1 : pan_level - 1;
    }
    if (g_fudge_tilt) {
        g_fudge_tilt = false;
        tilt_level = (tilt_level <= 0) ? tilt_level + 1 : tilt_level - 1;
    }

    if (pan_level == g_last_pan_level && tilt_level == g_last_tilt_level) {
        return;
    }
    g_last_pan_level = pan_level;
    g_last_tilt_level = tilt_level;

    bool left = pan_level > 0;
    bool right = pan_level < 0;
    bool up = tilt_level > 0;
    bool down = tilt_level < 0;

    if (localDebug) fprintf(stderr, "%s %s %s %s %d %d\n",
                            left ? "true" : "false",
                            right ? "true" : "false",
                            up ? "true" : "false",
                            down ? "true" : "false",
                            pan_level,
                            tilt_level);

    uint8_t pan_command = left ? 0x01 : right ? 0x2 : 0x3;
    uint8_t tilt_command = up ? 0x01 : down ? 0x2 : 0x3;

    uint8_t buf[9] = { 0x81, 0x01, 0x06, 0x01, 0x00, 0x00, pan_command, tilt_command, 0xFF };

    buf[4] = abs(pan_level) ?: 1; // Pan speed: 1 to 24 (0x18)
    buf[5] = abs(tilt_level) ?: 1; // Pan speed: 1 to 24 (0x18)

    if (localDebug) fprintf(stderr, "Sent packet %s\n", fmtbuf(buf, 9));

    send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
}

void sendExtendedPanTiltUpdatesOverVISCA(int sock, motionData_t *motionData) {
    const bool localDebug = false;
    int pan_level = (int)(motionData->xAxisPosition * (gMaxPanTiltValue + 0.9));
    int tilt_level = (int)(motionData->yAxisPosition * (gMaxPanTiltValue + 0.9));

    static int g_last_pan_level = 0;
    static int g_last_tilt_level = 0;

    static int count = 0;

    if (pan_level == g_last_pan_level && tilt_level == g_last_tilt_level) {
        return;
    }
    g_last_pan_level = pan_level;
    g_last_tilt_level = tilt_level;

    bool left = pan_level > 0;
    bool right = pan_level < 0;
    bool up = tilt_level > 0;
    bool down = tilt_level < 0;

    if (localDebug) fprintf(stderr, "%s %s %s %s %d %d\n",
                            left ? "true" : "false",
                            right ? "true" : "false",
                            up ? "true" : "false",
                            down ? "true" : "false",
                            pan_level,
                            tilt_level);

    uint8_t pan_command = left ? 0x01 : right ? 0x02 : 0x03;
    uint8_t tilt_command = up ? 0x01 : down ? 0x02 : 0x03;

    uint8_t buf[13] = { 0x81, 0x01, 0x06, 0x01, 0x00, 0x00, // Extended command.
                        0x00, 0x00, // Pan (big endian)
                        0x00, 0x00, // Tilt (big endian)
                        pan_command, tilt_command, 0xFF };

    buf[6] = (abs(pan_level) >> 8) & 0xff;
    buf[7] = pan_level & 0xff;
    buf[8] = (abs(tilt_level) >> 8) & 0xff;
    buf[9] = tilt_level & 0xff;

    if (localDebug) fprintf(stderr, "Sent packet %s\n", fmtbuf(buf, 9));

    send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
}

void sendManualExposureOverVISCA(int sock) {
    bool localDebug = enable_ptz_debugging;

    // Send this command only a few times on launch.
    static int count = 5;
    if (!count) return;
    count--;

    if (g_set_auto_exposure) {
        if (localDebug) fprintf(stderr, "Enabling automatic exposure.\n");
        uint8_t disablebuf[6] = { 0x81, 0x01, 0x04, 0x39, 0x00, 0xFF };
        send_visca_packet(sock, disablebuf, sizeof(disablebuf), VISCA_ACK_TIMEOUT, false, NULL);
        return;
    } else if (!(g_set_manual_iris || g_set_manual_gain || g_set_manual_gain)) {
        return;
    }

    if (localDebug) fprintf(stderr, "Enabling manual exposure.\n");
    uint8_t enablebuf[6] = { 0x81, 0x01, 0x04, 0x39, 0x03, 0xFF };
    send_visca_packet(sock, enablebuf, sizeof(enablebuf), VISCA_ACK_TIMEOUT, false, NULL);

    if (g_set_manual_iris) {
        if (localDebug) fprintf(stderr, "Setting iris position.\n");
        uint8_t irisbuf[9] = { 0x81, 0x01, 0x04, 0x4B, 0x00, 0x00,
                               (uint8_t)(g_manual_iris >> 4), (uint8_t)(g_manual_iris & 0xF), 0xFF };
        send_visca_packet(sock, irisbuf, sizeof(irisbuf), VISCA_ACK_TIMEOUT, false, NULL);
    }

    if (g_set_manual_gain) {
        if (localDebug) fprintf(stderr, "Setting gain position.\n");
        uint8_t gainbuf[9] = { 0x81, 0x01, 0x04, 0x4C, 0x00, 0x00,
                               (uint8_t)(g_manual_gain >> 4), (uint8_t)(g_manual_gain & 0xF), 0xFF };
        send_visca_packet(sock, gainbuf, sizeof(gainbuf), VISCA_ACK_TIMEOUT, false, NULL);
    }

    if (g_set_manual_gain) {
        if (localDebug) fprintf(stderr, "Setting gain position.\n");
        uint8_t gainbuf[9] = { 0x81, 0x01, 0x04, 0x4A, 0x00, 0x00,
                               (uint8_t)(g_manual_gain >> 4), (uint8_t)(g_manual_gain & 0xF), 0xFF };
        send_visca_packet(sock, gainbuf, sizeof(gainbuf), VISCA_ACK_TIMEOUT, false, NULL);
    }
}

void sendExposureCompensationOverVISCA(int sock) {
    if (!g_set_exposure_compensation) { return; }

    // Send this command only a few times on launch.
    static int count = 5;
    if (!count) return;
    count--;

    // Enable exposure compensation.
    uint8_t enablebuf[6] = { 0x81, 0x01, 0x04, 0x3E, (uint8_t)(g_exposure_compensation == 0 ? 0x03 : 0x02), 0xFF };
    send_visca_packet(sock, enablebuf, sizeof(enablebuf), VISCA_ACK_TIMEOUT, false, NULL);

    // Set compensation amount.
    if (g_exposure_compensation != 0) {
        uint8_t exposure_compensation_scaled = (uint8_t)(g_exposure_compensation + 5); // Range now 0 to 10
        uint8_t buf[9] = { 0x81, 0x01, 0x04, 0x4E, 0x00, 0x00, (uint8_t)((exposure_compensation_scaled >> 4) & 0xf),
                           (uint8_t)(exposure_compensation_scaled & 0xf), 0xFF };
        send_visca_packet(sock, buf, sizeof(buf), VISCA_ACK_TIMEOUT, false, NULL);
    }
}

bool get_ack(int sock, int timeout_usec);

ssize_t get_expected_response_length(uint8_t *buf, ssize_t bufsize) {
  bool valid = (bufsize > 2 && buf[0] == 0x81 && buf[1] == 0x09 && buf[bufsize - 1] == 0xFF);

  if (valid) {
    switch(bufsize) {
      case 5: /* Queries of length 5 */
        if (buf[2] == 0x04) {
          if (buf[3] == 0x07) {
                return 13;
          } else if (buf[3] == 0x46) {
                return 7;
          } else if (buf[3] == 0x47) {
                return 7;
          }
        } else if (buf[2] == 0x06 && buf[3] == 0x12) {
          return 13;
        }
        break;

      case 6: /* Queries of length 6 */
        if (buf[2] == 0x7E) {
          if (buf[3] == 0x01 && buf[4] == 0x0A) {
            return 4;
          } else if (buf[3] == 0x04 && buf[4] == 0x1A) {
            return 4;
          }
        }
      case 7: /* Queries of length 7 */
        if (buf[2] == 0x7E && buf[3] == 0x01 && buf[4] == 0x0A && buf[5] == 0x01) {
          return 4;
        }
        break;
      default:
        break;
    }
  } else {
    fprintf(stderr, "Not an inquiry.\n");
  }

  fprintf(stderr, "Unable to determine expected response length for inquiry %s\n", fmtbuf(buf, bufsize));

  return 0;
}

uint8_t *send_visca_inquiry(int sock, uint8_t *buf, ssize_t bufsize, int timeout_usec, ssize_t *responseLength) {
    bool localDebug = enable_verbose_debugging || false;
    if (sock == -1) {
        *responseLength = 0;
        return NULL;
    }

    int expectedLength = get_expected_response_length(buf, bufsize);

    __block pthread_cond_t semaphore = PTHREAD_COND_INITIALIZER;

    __block ssize_t localResponseLength = 0;
    __block uint8_t *responseBuf = NULL;
    __block uint32_t sequenceNumber = 0;  // Populated below.

    packetResponseHandler handler = ^(visca_network_packet_t packet) {
        int udp_offset = g_visca_use_udp ? 8 : 0;

        if (enable_verbose_debugging) {
          fprintf(stderr, "In packetResponseHandler %s\n", fmtbuf(packet->data, packet->length));
        }

        uint8_t *blockResponse = packet->data;

        // Verify that the response is for the correct message, and that the length
        // of the payload matches the packet length.
        if (g_visca_use_udp) {
            if (blockResponse[0] != 0x01 ||
                blockResponse[1] != 0x11 ||
                /* Bytes 2-3 are length. */
                ((packet->length - udp_offset) != ((uint32_t)blockResponse[2] << 8 | blockResponse[3])) ||
                /* Bytes 4 - 7 are sequence number, parsed already. */
                packet->sequenceNumber != sequenceNumber) {
                if (localDebug) {
                    fprintf(stderr, "%lf Unexpected response packet[1] %s\n", getVISCATimestamp(), fmtbuf(blockResponse, packet->length));
                    uint16_t payloadLength = (packet->length - udp_offset);
                    fprintf(stderr, "Expected %02x %02x %02x %02x %02x %02x %02x %02x\n", 1, 0x11, payloadLength >> 8, payloadLength & 0xff, sequenceNumber >> 24,
                            (sequenceNumber >> 16) & 0xff, (sequenceNumber >> 8) & 0xff, sequenceNumber & 0xff);
                }
                return false;
            }
        }

        // Verify that the packet length makes sense or that it is an error packet.
        bool isError = false;
        if (packet->length != (expectedLength + udp_offset)) {
            if (localDebug) {
              fprintf(stderr, "In packetResponseHandler expected length %lld got %lld\n",
                  (long long)expectedLength + udp_offset, (long long)packet->length);
              fprintf(stderr, "Command:  %s\n", fmtbuf(buf, bufsize));
              fprintf(stderr, "Response: %s\n", fmtbuf(packet->data, packet->length));
            }

            if (packet->length - udp_offset == 4) {
              if (packet->data[udp_offset] == 0x90 &&
                  packet->data[udp_offset + 1] == 0x60 &&
                  packet->data[udp_offset + 2] == 0x02 &&
                  packet->data[udp_offset + 3] == 0xFF) {
                if (localDebug) {
                  fprintf(stderr, "Is a syntax error packet.  Reporting the error.\n");
                }
                isError = true;
              }
            } else {
              if (localDebug) {
                fprintf(stderr, "Not a valid error packet.  Ignoring.\n");
              }
              return false;
            }
        } else if (blockResponse[1 + udp_offset] != 0x50) {
            if (localDebug) {
                fprintf(stderr, "Ignoring unexpected response packet[2] %s\n", fmtbuf(blockResponse, packet->length));
            }
            return false;
        }

        if (isError) {
          localResponseLength = 0;
          responseBuf = NULL;
        } else {
          localResponseLength = packet->length - udp_offset;
          responseBuf = (uint8_t *)malloc(localResponseLength);
          bcopy(blockResponse + udp_offset, responseBuf, localResponseLength);
        }

        if (localDebug) {
          fprintf(stderr, "pthread_cond_signal\n");
        }
        pthread_cond_signal(&semaphore);
        return true;
    };

    add_response_handler(handler);

    struct timespec abs_timeout;    
    clock_gettime(CLOCK_REALTIME, &abs_timeout);
    uint64_t nsec = abs_timeout.tv_nsec + ((uint64_t)timeout_usec) * 1000;
    abs_timeout.tv_nsec = nsec % NSEC_PER_SEC;
    abs_timeout.tv_sec += (nsec / NSEC_PER_SEC);

    if (send_visca_packet(sock, buf, bufsize, timeout_usec, true, &sequenceNumber) == -1) {
      if (localDebug) {
        fprintf(stderr, "send_visca_packet failed\n");
      }
      remove_response_handler(handler);
    } else {
      if (localDebug) {
        fprintf(stderr, "pthread_cond_timedwait\n");
      }
      pthread_mutex_t junk_mutex = PTHREAD_MUTEX_INITIALIZER;
      pthread_mutex_lock(&junk_mutex);
      if (pthread_cond_timedwait(&semaphore, &junk_mutex, &abs_timeout) < 0) {
        if (localDebug) {
          fprintf(stderr, "Read timeout.\n");
        }

        // Ensure that it doesn't get called and signal a dead semaphore.
        remove_response_handler(handler);

        *responseLength = 0;
        return NULL;
      }
      pthread_mutex_unlock(&junk_mutex);
      pthread_mutex_destroy(&junk_mutex);
      if (localDebug) {
        fprintf(stderr, "pthread_cond_timedwait done\n");
      }
    }

    pthread_cond_destroy(&semaphore);

    *responseLength = localResponseLength;
    return responseBuf;
}

// Sequence number must be provided for inquiries.  For all other packets, the value is
// ignored.
int send_visca_packet(int sock, uint8_t *buf, ssize_t bufsize, int timeout_usec, bool isInquiry,
                      uint32_t *outSequenceNumber) {
    // fprintf(stderr, "Sending VISCA packet on sock %d size %d\n", sock, (int)bufsize);
    if (sock == -1) {
        return -1;
    }

    uint8_t *copiedBuffer = (uint8_t *)malloc(bufsize);
    bcopy(buf, copiedBuffer, bufsize);

    pthread_mutex_lock(&g_networkMutex);
    static uint32_t visca_sequence_number = 0;
    uint32_t sequenceNumber = visca_sequence_number++;
    pthread_mutex_unlock(&g_networkMutex);

    if (outSequenceNumber != NULL) {
      *outSequenceNumber = sequenceNumber;
    }

    push_request(copiedBuffer, bufsize, isInquiry, sequenceNumber);

    if (isInquiry) {
        // Let the caller handle the read.
        return 0;
    }

    if (get_ack(sock, timeout_usec)) {
        return 0;
    } else {
        if (enable_verbose_debugging) {
            fprintf(stderr, "No ack.\n");
        }
        return -1;
    }
}

int send_visca_packet_raw(int sock, uint8_t *buf, ssize_t bufsize, bool isInquiry,
                          uint32_t sequenceNumber) {
    if (g_visca_use_udp) {
        ssize_t udpbufsize = bufsize + 8;
        uint8_t *udpbuf = (uint8_t *)malloc(udpbufsize);
        udpbuf[0] = 0x01;
        udpbuf[1] = isInquiry ? 0x10 : 0x00;
        udpbuf[2] = 0x00;
        udpbuf[3] = bufsize;
// 
        udpbuf[4] = sequenceNumber >> 24;
        udpbuf[5] = (sequenceNumber >> 16) & 0xff;
        udpbuf[6] = (sequenceNumber >> 8) & 0xff;
        udpbuf[7] = sequenceNumber & 0xff;
        bcopy(buf, udpbuf + 8, bufsize);
        if (sendto(sock, udpbuf, udpbufsize, 0,
               (sockaddr *)&g_visca_addr, sizeof(struct sockaddr_in)) != udpbufsize) {
            perror("write failed.");
            return -1;
        }

        if (enable_verbose_debugging) {
            fprintf(stderr, "Sent %s\n", fmtbuf(udpbuf, bufsize + 8));
        }
    } else {
        if (write(sock, buf, bufsize) != bufsize) {
            perror("write failed.");
            return -1;
        }
    }
    return 0;
}

bool get_ack(int sock, int timeout_usec) {
    bool localDebug = enable_verbose_debugging || false;
    int udp_offset = g_visca_use_udp ? 8 : 0;
    if (enable_verbose_debugging) {
        fprintf(stderr, "in get_ack\n");
    }
    ssize_t len;

    double startTime = getVISCATimestamp();

    // IMPORTANT: Take and release the lock inside this loop to avoid blocking
    // the network thread.
    bool found_5 = false, found_4 = false;
    while (getVISCATimestamp() < startTime + 0.2 && !found_5) {
        pthread_mutex_lock(&g_networkMutex);
        visca_network_packet_t packet = g_visca_indata->head;
        int ack_type = -1;

        while (packet && !(found_4 && found_5)) {
            ack_type = ackType(packet);
            if (ack_type == 5 && !found_5) {
                found_5 = true;
                if (enable_verbose_debugging) {
                    fprintf(stderr, "DEQUEUE ACK 5 %s\n", fmtbuf(packet->data, packet->length));
                }
                packet = deletePacketFromQueue(packet, g_visca_indata);
            }
            if (ack_type == 4 && !found_4) {
                found_4 = true;
                if (enable_verbose_debugging) {
                    fprintf(stderr, "DEQUEUE ACK 4 %s\n", fmtbuf(packet->data, packet->length));
                }
                packet = deletePacketFromQueue(packet, g_visca_indata);
            } else {
                packet = packet->next;
            }
        }

        pthread_mutex_unlock(&g_networkMutex);
    }

    return found_5;
}

#pragma mark - PTZ Core

void sendPTZUpdates(NDIlib_recv_instance_t pNDI_recv) {
    motionData_t copyOfMotionData = getMotionData();

    // We want to send a "set position X" or "retrieve position X" message only once.  To do this, we
    // keep track of the last set/retrieve command sent, and if the value hasn't changed, we zero
    // the value that we send to the rest of the app.
    static motionData_t lastMotionData = { 0.0, 0.0, 0.0, 0, 0 };

    if (enable_ptz_debugging && copyOfMotionData.zoomPosition != 0) {
        fprintf(stderr, "zSpeed: %f ", copyOfMotionData.zoomPosition);
    }

    if (!enable_visca_ptz || !visca_running || g_visca_sock == -1) {
        // Certain major-brand PTZ cameras have a defect where they sometimes
        // lose VISCA commands and think that they have changed the speed when
        // they actually have not.  If the camera is still moving and should be
        // stopped, send a single nonzero motion command and then immediately
        // send a stop command again to work around the bug.
        if (g_fudge_zoom && copyOfMotionData.zoomPosition == 0) {
            g_fudge_zoom = false;
            copyOfMotionData.zoomPosition = 0.05;
        }

        NDIlib_recv_ptz_zoom_speed(pNDI_recv, -copyOfMotionData.zoomPosition);
        g_last_zoom_level = copyOfMotionData.zoomPosition > 0 ? ceil(copyOfMotionData.zoomPosition * 20)
            : floor(copyOfMotionData.zoomPosition * 20);
    }

    if (!enable_visca_ptz || !visca_running || g_visca_sock == -1) {
        if (enable_verbose_debugging || enable_ptz_debugging) {
            fprintf(stderr, "NDI MODE: %f, %f\n", copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
        }

        // Certain major-brand PTZ cameras have a defect where they sometimes
        // lose VISCA commands and think that they have changed the speed when
        // they actually have not.  If the camera is still moving and should be
        // stopped, send a single nonzero motion command and then immediately
        // send a stop command again to work around the bug.
        if (g_fudge_pan && copyOfMotionData.xAxisPosition == 0) {
            g_fudge_pan = false;
            copyOfMotionData.xAxisPosition = 0.05;
        }
        if (g_fudge_tilt && copyOfMotionData.yAxisPosition == 0) {
            g_fudge_tilt = false;
            copyOfMotionData.yAxisPosition = 0.05;
        }

        NDIlib_recv_ptz_pan_tilt_speed(pNDI_recv, copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
        g_last_pan_level = copyOfMotionData.xAxisPosition > 0 ? ceil(copyOfMotionData.xAxisPosition * 20)
            : floor(copyOfMotionData.xAxisPosition * 20);
        g_last_tilt_level = copyOfMotionData.yAxisPosition > 0 ? ceil(copyOfMotionData.yAxisPosition * 20)
            : floor(copyOfMotionData.yAxisPosition * 20);

        if (enable_ptz_debugging) {
            if (copyOfMotionData.xAxisPosition != 0 || copyOfMotionData.yAxisPosition != 0) {
                fprintf(stderr, "xSpeed: %f, ySpeed; %f ", copyOfMotionData.xAxisPosition, copyOfMotionData.yAxisPosition);
            }
        }
    }
    if (copyOfMotionData.xAxisPosition != 0 || copyOfMotionData.yAxisPosition != 0 || copyOfMotionData.zoomPosition != 0) {
        g_is_loading_preset = false;
    }
    if (copyOfMotionData.retrievePositionNumber > 0 &&
        copyOfMotionData.retrievePositionNumber != lastMotionData.retrievePositionNumber) {
        fprintf(stderr, "Retrieving position %d\n", copyOfMotionData.retrievePositionNumber);
        g_is_loading_preset = true;
        if (use_visca_for_presets) {
            sendVISCALoadPreset((uint8_t)copyOfMotionData.retrievePositionNumber, g_visca_sock);
        } else {
            NDIlib_recv_ptz_recall_preset(pNDI_recv, copyOfMotionData.retrievePositionNumber, recallSpeedNDI());
        }
    } else if (copyOfMotionData.storePositionNumber > 0 &&
               copyOfMotionData.storePositionNumber != lastMotionData.storePositionNumber) {
        fprintf(stderr, "Storing position %d\n", copyOfMotionData.storePositionNumber);
        if (use_visca_for_presets) {
            sendVISCASavePreset((uint8_t)copyOfMotionData.storePositionNumber, g_visca_sock);
        } else {
            NDIlib_recv_ptz_store_preset(pNDI_recv, copyOfMotionData.storePositionNumber);
        }
    }
    lastMotionData = copyOfMotionData;
}

// Return a value from 0 to 1.
double recallSpeedNDI(void) {
  // For now, return maximum possible speed.
  return 1.0;
}

int recallSpeedVISCA(void) {
  int value = (recallSpeedNDI() * 23.0) + 1.5;
  return (value > 24) ? 24 : (value < 1) ? 1 : value;
}

#pragma mark - Button and joystick input

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
    #ifndef ENABLE_FILES_FOR_BUTTON_TESTING
        if (!io_expander) return 0;
        int pin = pinNumberForAxis(axis);
        int rawValue = input(io_expander, pin, 0.001) - 2048;
        return scaleAxisValue(axis, rawValue);
    #else  // ENABLE_FILES_FOR_BUTTON_TESTING
        bool localDebug = false;
        char *filename;
        safe_asprintf(&filename, "/var/tmp/axis.%d", axis);
        FILE *fp = fopen(filename, "r");
        free(filename);
        float value = 0.0;
        if (fp) {
            fscanf(fp, "%f\n", &value);
            fclose(fp);
            if (enable_ptz_debugging || localDebug) {
                fprintf(stderr, "Returning %f for axis %d\n", value, axis);
            }
            return value;
        }
        if (enable_verbose_debugging || localDebug) {
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
bool debounce(int buttonNumber, bool value, motionData_t *motionData, int debounceCount);
bool readButton(int buttonNumber, motionData_t *motionData) {
    #ifdef __linux__
        if (!io_expander) return 0;
        int pin = pinNumberForButton(buttonNumber);
        int rawValue = input(io_expander, pin, 0.001);
        bool value = (rawValue == LOW);  // If logic low (grounded), return true.

        if (enable_button_debugging) {
            fprintf(stderr, "button %d: raw: %d scaled: %s\n", buttonNumber, rawValue, value ? "true" : "false");
        }
    #else  // ! __linux__
        bool value = false;
        char *filename;
        safe_asprintf(&filename, "/var/tmp/button.%d", buttonNumber);
        FILE *fp = fopen(filename, "r");
        free(filename);
        if (fp) {
            fclose(fp);
            if (enable_button_debugging) {
                fprintf(stderr, "Returning true for button %d\n", buttonNumber);
            }
            value = true;
        } else {
            if (enable_button_debugging) {
                fprintf(stderr, "Returning false for button %d\n", buttonNumber);
            }
            value = false;
        }
    #endif  // __linux__

    return debounce(buttonNumber, value, motionData, 3);
}

// Don't change values until the value has been consistently high or low
// for `debounceCount` cycles.
bool debounce(int buttonNumber, bool value, motionData_t *motionData, int debounceCount) {
    // Debounce the value.
    const bool localDebug = false;

    // Previous value comes from the last call to debounce (i.e. the last
    // time the button was read).
    bool buttonChanged = motionData->previousValue[buttonNumber] != value;

    if (buttonChanged) {
      motionData->previousValue[buttonNumber] = value;
      motionData->debounceCounter[buttonNumber] = 1;
      if (localDebug) {
        fprintf(stderr, "Debounce[%d] = 1\n", buttonNumber);
      }
    } else if (motionData->debounceCounter[buttonNumber] < (debounceCount - 1)) {
      motionData->debounceCounter[buttonNumber]++;
      if (localDebug) {
        fprintf(stderr, "Debounce[%d]++ -> %d\n", buttonNumber,
               motionData->debounceCounter[buttonNumber]);
      }
    } else {
      // Update the value to return.
      motionData->currentValue[buttonNumber] = value;
      if (localDebug) {
        fprintf(stderr, "Current[%d] -> %s\n", buttonNumber,
                value ? "true" : "false");
      }
    }
    if (localDebug) {
      fprintf(stderr, "Debounce return[%d] -> %s\n", buttonNumber,
            motionData->currentValue[buttonNumber] ? "true" : "false");
    }

    return motionData->currentValue[buttonNumber];
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
    // Copy the old data, for debounce reasons.
    motionData_t newMotionData = getMotionData();

    // Update the analog axis values.
    newMotionData.xAxisPosition = readAxisPosition(kPTZAxisX);
    newMotionData.yAxisPosition = readAxisPosition(kPTZAxisY);
    newMotionData.zoomPosition = readAxisPosition(kPTZAxisZoom);
    if (enable_ptz_debugging) {
        fprintf(stderr, "\n");
    }

    // Determine whether the set button is down.
    newMotionData.setButtonDown = readButton(BUTTON_SET, &newMotionData);

    // Print a debug message when the user presses or releases the set button,
    // but only once per transition.
    static bool showedInitialState = false;
    static bool lastSetButtonDown = false;
    if (!showedInitialState || (newMotionData.setButtonDown != lastSetButtonDown)) {
        if (enable_button_debugging) {
            fprintf(stderr, "Set button %s\n", newMotionData.setButtonDown ? "DOWN" : "UP");
        }
        showedInitialState = true;
    }
    if (newMotionData.setButtonDown && !lastSetButtonDown) {
        newMotionData.setMode = !newMotionData.setMode;
    }
    lastSetButtonDown = newMotionData.setButtonDown;

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
        if (readButton(i, &newMotionData)) {
            if (newMotionData.setMode) {
                if (enable_button_debugging) {
                    fprintf(stderr, "Store position %d\n", i);
                }
                newMotionData.storePositionNumber = i;
            } else {
                if (enable_button_debugging) {
                    fprintf(stderr, "Retrieve position %d\n", i);
                }
                newMotionData.retrievePositionNumber = i;
            }
        } else {
            if (enable_button_debugging) {
                fprintf(stderr, "Inactive position %d\n", i);
            }
        }
    }

    updateLights(&newMotionData);

    setMotionData(newMotionData);
    if (visca_running) {
        sendPTZUpdatesOverVISCA(&newMotionData);
    }
}

void updateLights(motionData_t *motionData) {
    static bool localDebug = enable_button_debugging || false;
    static int blinkingButtons = 0;  // bitmap
    static int litButtons = 0;       // bitmap
    static int blinkCounter = 0;
    static bool holdAfterBlink = true;

    if (localDebug) fprintf(stderr, "Pos: %d\n", motionData->retrievePositionNumber);

    const int setButtonMask = 0b00000001;
    const int mainButtonMask = 0b00111110;

    litButtons &= ~setButtonMask;
    litButtons |= motionData->setMode;
    if (fabs(motionData->xAxisPosition) > 0 ||
        fabs(motionData->yAxisPosition) > 0 ||
        fabs(motionData->zoomPosition) > 0) {
            // Reset.
            if (localDebug) fprintf(stderr, "LIGHTS: Motion\n");
            litButtons &= ~mainButtonMask;
            holdAfterBlink = false;
    } else {
        if (motionData->retrievePositionNumber) {
            if (localDebug) fprintf(stderr, "LIGHTS: Retrieve %d\n", motionData->retrievePositionNumber);
            blinkCounter = 0;
            litButtons &= ~mainButtonMask;
            litButtons |= (1 << motionData->retrievePositionNumber);
        } else if (motionData->storePositionNumber > 0) {
            if (localDebug) fprintf(stderr, "LIGHTS: Store %d\n", motionData->storePositionNumber);
            blinkingButtons |= 1 << motionData->storePositionNumber;
            blinkCounter = PULSES_PER_BLINK * 5;
        }
    }

    if (blinkCounter) {
        blinkCounter--;
        if (localDebug) fprintf(stderr, "LIGHTS: Blink: %d\n", blinkingButtons);
        if ((blinkCounter / PULSES_PER_BLINK) % 2 == 0) {
            litButtons |= blinkingButtons;
        } else {
            litButtons &= ~blinkingButtons;
        }
        if (blinkCounter == 0 && !holdAfterBlink) {
            litButtons &= ~blinkingButtons;
        }
    } else {
        blinkingButtons = 0;
    }

    const char *onoff[2] = { "OFF", "ON " };
    if (localDebug) fprintf(stderr, "White: %s Red: %s Yellow: %s Green: %s Blue: %s Black: %s\n",
            onoff[(bool)(litButtons & 0b1)],
            onoff[(bool)(litButtons & 0b10)],
            onoff[(bool)(litButtons & 0b100)],
            onoff[(bool)(litButtons & 0b1000)],
            onoff[(bool)(litButtons & 0b10000)],
            onoff[(bool)(litButtons & 0b100000)]);

#if __linux__
    if (!g_use_on_screen_lights) {
        cc_gpio_write(LED_PIN_WHITE,  (bool)(litButtons & 0b1));      // 0 (white)
        cc_gpio_write(LED_PIN_RED,  (bool)(litButtons & 0b10));       // 1 (red)
        cc_gpio_write(LED_PIN_YELLOW, (bool)(litButtons & 0b100));    // 2 (yellow)
        cc_gpio_write(LED_PIN_GREEN, (bool)(litButtons & 0b1000));    // 3 (green)
        cc_gpio_write(LED_PIN_BLUE, (bool)(litButtons & 0b10000));    // 4 (blue)
        cc_gpio_write(LED_PIN_PURPLE, (bool)(litButtons & 0b100000)); // 5 (black)
    } else {
#endif // __linux__
        motionData->light[0] = (bool)(litButtons & 0b1);      // 0 (white)
        motionData->light[1] = (bool)(litButtons & 0b10);     // 1 (red)
        motionData->light[2] = (bool)(litButtons & 0b100);    // 2 (yellow)
        motionData->light[3] = (bool)(litButtons & 0b1000);   // 3 (green)
        motionData->light[4] = (bool)(litButtons & 0b10000);  // 4 (blue)
        motionData->light[5] = (bool)(litButtons & 0b100000); // 5 (black)
#if __linux__
    }
#endif // __linux__

    struct timespec current_wallclock_time;
    clock_gettime(CLOCK_REALTIME, &current_wallclock_time);
    // Add at most a second if the seconds are different.  If we've lost more than a second of video,
    // things aren't working anyway.
    uint64_t diff = ((current_wallclock_time.tv_sec != last_frame_time.tv_sec) ? 1000000000 : 0) +
        current_wallclock_time.tv_nsec - last_frame_time.tv_nsec;

    g_camera_malfunctioning = diff > 100000000;  // Scream after losing 3 frames.

#if __linux__
    if (!g_use_on_screen_lights) {
        // Program/RGB red
        cc_gpio_write(LED_PIN_RGB_RED, g_camera_active && !g_camera_malfunctioning);

        // Preview/RGB green
        cc_gpio_write(LED_PIN_RGB_GREEN, g_camera_preview && !g_camera_malfunctioning);

        // Malfunction/RGB blue
        cc_gpio_write(LED_PIN_RGB_BLUE, g_camera_malfunctioning);
    }
#endif // __linux__


    // fprintf(stderr, "Preview: %s Program: %s\n",
            // onoff[g_camera_preview && !g_camera_malfunctioning],
            // onoff[g_camera_active && !g_camera_malfunctioning]);

    // fprintf(stderr, "Camera: %s\n", g_camera_malfunctioning ? "MALFUNCTIONING" : "normal");

    // fprintf(stderr, "%d %d %d %d\n", current_wallclock_time.tv_nsec, last_frame_time.tv_nsec, current_wallclock_time.tv_sec, last_frame_time.tv_nsec);

    // Pinout for lights (last 12 pins):
    // GPIO 5 - White light  | Ground - NC
    // GPIO 6 - Red light    | GPIO 12 - Black
    // GPIO13 - Yellow light | Ground - NC
    // GPIO19 - Green light  | GPIO 16 - RGB Red
    // GPIO26 - Blue light   | GPIO 20 - RGB Green
    // Ground - All LEDs     | GPIO 21 - RGB Blue
}

bool configureGPIO(void) {
#ifdef __linux__
#ifdef USE_MRAA
    if (mraa_init() != MRAA_SUCCESS) {
        return false;
    }
    bzero((void *)soft_pwm_enabled, sizeof(soft_pwm_enabled));
    bzero((void *)soft_pwm_value, sizeof(soft_pwm_value));
    bzero(gpio_contexts, sizeof(gpio_contexts));
#else
    g_pig = pigpio_start(NULL, NULL);
    if (g_pig < 0) {
        fprintf(stderr, "Could not connect to pigpiod daemon.  (Try sudo systemctl enable pigpiod)\n");
        return false;
    }
#endif

    if (set_mode(g_pig, LED_PIN_WHITE, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_RED, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_YELLOW, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_GREEN, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_BLUE, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_PURPLE, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_RGB_RED, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_RGB_GREEN, PI_OUTPUT))
        return false;
    if (set_mode(g_pig, LED_PIN_RGB_BLUE, PI_OUTPUT))
        return false;
#endif // __linux__

    return true;
}

#ifdef __linux__
uint8_t dutyCycle(int pin) {
    double adjustment = 1.0;
    switch(pin) {
        case LED_PIN_WHITE: // Button 0 (Set)
            adjustment = WHITE_DUTY_CYCLE;
            break;
        case LED_PIN_RED: // Button 1
            adjustment = RED_DUTY_CYCLE;
            break;
        case LED_PIN_YELLOW: // Button 2
            adjustment = YELLOW_DUTY_CYCLE;
            break;
        case LED_PIN_GREEN: // Button 3
            adjustment = GREEN_DUTY_CYCLE;
            break;
        case LED_PIN_BLUE: // Button 4
            adjustment = BLUE_DUTY_CYCLE;
            break;
        case LED_PIN_PURPLE: // Button 5
            adjustment = PURPLE_DUTY_CYCLE;
            break;
        case LED_PIN_RGB_RED:  // Tally Program
            adjustment = RGB_RED_DUTY_CYCLE;
            break;
        case LED_PIN_RGB_GREEN:  // Tally Preview
            adjustment = RGB_GREEN_DUTY_CYCLE;
            break;
        case LED_PIN_RGB_BLUE:  // Tally Malfunction
            adjustment = RGB_BLUE_DUTY_CYCLE;
            break;
        default:
            adjustment = 1.0;
    }
    int64_t duty_cycle = (base_duty_cycle * adjustment);
    if (duty_cycle < 0) return 0;
    if (duty_cycle > 255) return 255;
    return (uint8_t)duty_cycle;
}

// A variant of gpio_write that automatically sets a
// duty cycle on the pin instead of just turning it on.
bool cc_gpio_write(unsigned gpio, unsigned level) {
#if USE_MRAA
    return set_PWM_dutycycle(g_pig, gpio, level ? dutyCycle(gpio) : 0);
#else  // USE_MRAA
    if (level) {
        return set_PWM_dutycycle(g_pig, gpio, dutyCycle(gpio));
    } else {
        return gpio_write(g_pig, gpio, level);
    }
#endif  // USE_MRAA
}
#endif // __linux__

#endif // !DEMO_MODE

void setMotionData(motionData_t newMotionData) {
    /*
     * Lock the motion data mutex and update the motion data so that the NDI code
     * running on the main thread can send it to the camera.
     */
    pthread_mutex_lock(&g_motionMutex);
    g_motionData = newMotionData;
    pthread_mutex_unlock(&g_motionMutex);
}

motionData_t getMotionData() {
    motionData_t newMotionData;
    /*
     * Lock the motion data mutex and update the motion data so that the NDI code
     * running on the main thread can send it to the camera.
     */
    pthread_mutex_lock(&g_motionMutex);
    newMotionData = g_motionData;
    pthread_mutex_unlock(&g_motionMutex);
    return newMotionData;
}

// Run this computation up to 2000 times per second.  In practice, the NDI side
// doesn't need the data that quickly, but the VISCA side runs slowly enough that
// a longer delay makes it not work well.  However, no delay causes the NDI side
// to drop pan commands, resulting in the camera continuing to pan indefinitely
// even after you stop panning.
//
// The goal was to do this only 200 times per second.  By only doing this
// periodically, we limit the amount of CPU overhead, leaving more cycles to do
// the actual H.264 or H.265 decoding.
void *runPTZThread(void *argIgnored) {
    while (true) {
#ifdef DEMO_MODE
        demoPTZValues();
#else  // !DEMO_MODE

#ifdef __linux__
        if (io_expander != NULL) {
#endif  // !__linux__

            updatePTZValues();

#ifdef __linux__
        }
#endif  // !__linux__

        // The I/O expander is slow enough that we don't
        usleep(5000);
#endif  // DEMO_MODE
    }
}

#pragma mark - Source management

void free_receiver_item(receiver_array_item_t receiver_item) {
    free(receiver_item->name);
    free(receiver_item->thread_data->stream_name);
    free(receiver_item->thread_data);
    free(receiver_item);
}

void truncate_name_before_ip(char *name) {
    for (char *pos = &name[strlen(name) - 1] ; pos >= name; pos--) {
        if (*pos == ',') {
            *pos = '\0';
            break;
        }
    }

    // If there was no comma, truncate the final close parenthesis to
    // allow matches across firmware changes that remove the IP address
    // from the name.
    ssize_t lastpos = strlen(name) - 1;
    if (lastpos >= 0 && name[lastpos] == ')') {
            name[lastpos] = '\0';
    }
}

bool source_name_compare(const char *name1, const char *name2, bool use_fallback) {
    if (!use_fallback) {
        return !strcmp(name1, name2);
    }

    char *truncname1 = NULL, *truncname2 = NULL;
    safe_asprintf(&truncname1, "%s", name1);
    safe_asprintf(&truncname2, "%s", name2);
    truncate_name_before_ip(truncname1);
    truncate_name_before_ip(truncname2);

    bool retval = !strcmp(truncname1, truncname2);
    if (enable_verbose_debugging) {
        fprintf(stderr, "CMP \"%s\" ?= \"%s\"\n", name1, name2);
        fprintf(stderr, "CMP \"%s\" ?= \"%s\" : %s\n", truncname1, truncname2,
                retval ? "true" : "false");
    }
    free(truncname1);
    free(truncname2);
    return retval;
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
            const char *found_source_name = p_sources[i].p_ndi_name;
            bool is_shown = sourceIsActiveForName(found_source_name, g_shown_sources);
            if (!is_shown) {
                fprintf(stderr, "    \"%s\"\n", found_source_name);
                receiver_array_item_t receiver_item = new_receiver_array_item();
                safe_asprintf(&receiver_item->name, "%s", found_source_name);
                receiver_item->next = g_shown_sources;
                g_shown_sources = receiver_item;
            }
        }
    }
    return -1;
}

bool sourceIsActiveForName(const char *source_name, receiver_array_item_t array) {
    for (receiver_array_item_t check_item = array;
        check_item != NULL;
        check_item = check_item->next) {
        if (!strcmp(check_item->name, source_name)) {
            return true;
        }
    }
    return false;
}

#pragma mark - Formatting

char fmtnibble(uint8_t nibble) {
    if (nibble <= 9) return '0' + nibble;
    return 'A' - 10 + nibble;
}

char *fmtbuf(uint8_t *buf, ssize_t bufsize) {
    static thread_local char *retval = NULL;
    if (retval != NULL) {
        free(retval);
        retval = NULL;
    }
    retval = (char *)malloc(MAX(1, (bufsize * 3)));
    if (bufsize) {
        for (ssize_t i = 0 ; i < bufsize; i++) {
            retval[i * 3] = fmtnibble(buf[i] >> 4);
            retval[(i * 3) + 1] = fmtnibble(buf[i] & 0xf);
            retval[(i * 3) + 2] = ' ';
        }
        retval[(bufsize * 3) - 1] = '\0';
    } else {
        retval[0] = '\0';
    }
    return retval;
}

#pragma mark - Tests

#ifdef DEMO_MODE
void demoSendPTZ(void) {
#ifdef PTZ_TESTING
    motionData_t motionData = getMotionData();
    sendPTZUpdatesOverVISCA(&motionData);
#endif
}

void demoPTZValues(void) {
    motionData_t motionData;
    bzero(&motionData, sizeof(motionData));

    // Move the camera for one second at a time.
    motionData.xAxisPosition = 1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.xAxisPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.yAxisPosition = 1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.yAxisPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.zoomPosition = 1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000); fprintf(stderr, "ZOOM 1.0");
    motionData.zoomPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000); fprintf(stderr, "ZOOM 0");

    // Store in position 1.
    motionData.storePositionNumber = 1; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);

    motionData.xAxisPosition = -1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.xAxisPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.yAxisPosition = -1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.yAxisPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.zoomPosition = -1.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000); fprintf(stderr, "ZOOM -1.0");
    motionData.zoomPosition = 0.0; setMotionData(motionData); demoSendPTZ(); usleep(1000000); fprintf(stderr, "ZOOM 0");

    // Store in position 2.
    motionData.storePositionNumber = 2; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);

    motionData.retrievePositionNumber = 1; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);

    motionData.retrievePositionNumber = 2; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
    motionData.storePositionNumber = 0; setMotionData(motionData); demoSendPTZ(); usleep(1000000);
}
#endif

void testDebounce(void);
void runUnitTests(void) {
#ifndef DEMO_MODE
    testDebounce();
#endif  // DEMO_MODE
}

void testPin(int pin);
void runLightTests(void) {
#if __linux__
  fprintf(stderr, "Testing lights.\n");

  testPin(LED_PIN_WHITE);
  testPin(LED_PIN_RED);
  testPin(LED_PIN_YELLOW);
  testPin(LED_PIN_GREEN);
  testPin(LED_PIN_BLUE);
  testPin(LED_PIN_PURPLE);
  testPin(LED_PIN_RGB_RED);
  testPin(LED_PIN_RGB_GREEN);
  testPin(LED_PIN_RGB_BLUE);
#endif  // __linux__
}

#if __linux__
void testPin(int pin) {
  for (int i = 0 ; i < 250; i += 50) {
    set_PWM_dutycycle(0, pin, i);
    usleep(1000000);
  }
  set_PWM_dutycycle(0, pin, 255);
  usleep(1000000);
  set_PWM_dutycycle(0, pin, 0);
}
#endif  // __linux__

#ifndef DEMO_MODE
// bool debounce(int buttonNumber, bool value, motionData_t *motionData, int debounceCount);
void testDebounce(void) {
    motionData_t motionData;
    memset(&motionData, 0, sizeof(motionData));

    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(debounce(0, true, &motionData, 5));
    assert(debounce(0, false, &motionData, 5));
    assert(debounce(0, true, &motionData, 5));
    assert(debounce(0, true, &motionData, 5));
    assert(motionData.debounceCounter[0] == 2);
    assert(debounce(0, false, &motionData, 3));
    assert(debounce(0, false, &motionData, 3));
    assert(!debounce(0, false, &motionData, 3));
    assert(!debounce(0, true, &motionData, 3));
    assert(!debounce(0, false, &motionData, 3));
    assert(!debounce(0, false, &motionData, 3));
    assert(motionData.debounceCounter[0] == 2);
    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(!debounce(0, true, &motionData, 5));
    assert(debounce(0, true, &motionData, 5));
    assert(!debounce(1, true, &motionData, 5));
    assert(!debounce(1, true, &motionData, 5));
    assert(!debounce(1, true, &motionData, 5));
    assert(!debounce(1, true, &motionData, 5));
    assert(debounce(1, true, &motionData, 5));
    assert(debounce(1, true, &motionData, 5));
    assert(debounce(1, true, &motionData, 5));
    assert(debounce(0, false, &motionData, 5));
    assert(debounce(0, true, &motionData, 5));
    assert(motionData.debounceCounter[1] == 4);
}
#endif  // DEMO_MODE

#ifdef USE_MRAA

// This leaks, so don't do it too much.
int set_PWM_dutycycle(__attribute__ ((unused)) int unused,
                  unsigned gpioPin, unsigned dutyCycle) {
  soft_pwm_value[gpioPin] = dutyCycle;
  soft_pwm_enabled[gpioPin] = true;
  return true;
}

int gpio_write(int pi, unsigned gpioPin, unsigned level) {
  if (gpio_contexts[gpioPin] == NULL) {
    gpio_contexts[gpioPin] = mraa_gpio_init(gpioPin);
  }
  return mraa_gpio_write(gpio_contexts[gpioPin], level ? 1 : 0) == MRAA_SUCCESS;
}

int set_mode_sub(int pi, unsigned gpioPin, unsigned mode) {
  fprintf(stderr, "Initializing pin %d\n", gpioPin);
  if (gpio_contexts[gpioPin] == NULL) {
    gpio_contexts[gpioPin] = mraa_gpio_init(gpioPin);
  }
  if (gpio_contexts[gpioPin] == NULL) {
    fprintf(stderr, "Could not initialize pin %d: unable to create context.\n", gpioPin);
  }
  assert(mode == PI_OUTPUT);

  bool retval = (mraa_gpio_dir(gpio_contexts[gpioPin], MRAA_GPIO_OUT) != MRAA_SUCCESS);
  if (retval) {
    fprintf(stderr, "Could not initialize pin %d: Unable to set direction.\n", gpioPin);
  }
  return retval;
}

// The set_mode call we are emulating returns 0 on success, so we must do so as well.
// Unfortunately, when we create a GPIO pin entry, it takes a bit of time for it to get
// the right group and permissions, so we may have to retry a few times.
int set_mode(int pi, unsigned gpioPin, unsigned mode) {
  int retval = 1;
  for (int i = 0; i < 10; i++) {
    retval = set_mode_sub(pi, gpioPin, mode);
    if (retval == 0) {
      return retval;
    }
    usleep(100000);
  }
  return retval;
}

void *runPWMThread(void *argIgnored) {
  int pos = 0;

  while (!exit_app) {
    for (int pin = 1; pin <= MAX_PINS; pin++) {
      if (soft_pwm_enabled[pin]) {
        if (pos == 0) {
          gpio_write(0, pin, (soft_pwm_value[pin] > 0) ? 1 : 0);
        } else if (soft_pwm_value[pin] == pos) {
          gpio_write(0, pin, 0);
        }
      }
    }
    pos = (pos + 1) % 255;
  }

  // Clean up quickly and terminate before we can cause a segfault.
  for (int pin = 1; pin <= MAX_PINS; pin++) {
    if (soft_pwm_enabled[pin]) {
      gpio_write(0, pin, 0);
    }
  }

  return NULL;
}

#endif

float scaleAxisValue(int axis, int rawValue) {
  // In theory, the range is -2048 to 2048.  Unfortunately, analog
  // potentiometers aren't at all consistent about their centering
  // so to avoid problems, treat everything from -300 to 300 as zero,
  // and everything from 1700 up as maximum speed.

  double minThreshold = 300;
  double maxThreshold = 1700;
  double powerMultiplier = 3.0;

  double sign = (rawValue > 0) ? 1 : -1;

  // Subtract off the minimum and clamp at zero.
  double absRawValue = abs(rawValue) - minThreshold;
  if (absRawValue < 0) absRawValue = 0;

  // Divide by the range to scale from 0 to 1 (float).
  double value = absRawValue / (1.0 * maxThreshold - minThreshold);

  if (value > 1.0) value = 1.0;

  // Use a power computation to push the point of fast motion out towards the extremes.
  value = pow(value, powerMultiplier);

  // Always treat zero as positive.
  if (value != 0.0) value *= sign;

  if (enable_ptz_debugging) {
      fprintf(stderr, "axis %d: raw: %d scaled: %f ", axis, rawValue, value);
  }

  // If VISCA is enabled, even if we're using NDI for PTZ control, we can still poll the zoom
  // position and be smart about the pan and tilt speed.
  if (enable_visca) {
    // Range is 0 to 1, where 1 is zoomed all the way in, with 20x being about 0x4000, 30x about
    // 0x6000, etc., or at least that seems to be the typical range.  After scaling, this is
    // 0.25 for 20x and 0.375 for 30x.
    //
    // Based on this, a zoom position of 0 (zoomed out) has no effect, and a zoom position of
    // 0.375 (30x) is capped at about 25% of normal speed.
    //
    // This also caps the minimum speed at 0.1x so that no matter how far in you zoom, you don't
    // lose the ability to pan and tilt, and caps the maximum scale at 1.0 in case something is
    // very, very wrong.
    double scale = MIN(MAX(1 - (2 * gCurrentZoomPosition), 0.1), 1.0);
    if (enable_verbose_debugging) {
        fprintf(stderr, "Current zoom position: %lf\nScale: %lf\n", gCurrentZoomPosition, scale);
    }
    value *= scale;
  }

  return value;
}

// This should be called ONLY under a lock.  Returns the packet after the deleted packet.
visca_network_packet_t deletePacketFromQueue(visca_network_packet_t packet, visca_network_queue_t queue) {

  visca_network_packet_t nextPacket = packet->next;
  visca_network_packet_t previousPacket = packet->prev;

  if (previousPacket == NULL) {
    queue->head = nextPacket;
  } else {
    previousPacket->next = nextPacket;
  }
  if (nextPacket == NULL) {
    queue->tail = previousPacket;
  } else {
    nextPacket->prev = previousPacket;
  }
  free(packet->data);
  free(packet);
  return nextPacket;
}

visca_network_packet_t newVISCANetworkPacket(uint8_t *data, size_t length, bool isInquiry, 
                                             uint32_t sequenceNumber) {
  visca_network_packet_t packet = (visca_network_packet_t)malloc(sizeof(*packet));
  packet->length = length;
  packet->data = data;
  packet->timestamp = getVISCATimestamp();
  packet->isInquiry = isInquiry;
  packet->sequenceNumber = sequenceNumber;
  packet->prev = NULL;
  packet->next = NULL;
  return packet;
}

void printQueue(visca_network_queue_t queue) {
  visca_network_packet_t packet = queue->head;
  while (packet) {
    fprintf(stderr, "[Packet length %zu]\n", packet->length);
    packet = packet->next;
  }
}

uint32_t parseSequenceNumber(uint8_t *data, size_t length) {
  if (g_visca_use_udp) {
    return (uint32_t)data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
  } else {
    return 42;  // Unused anyway.
  }
}

void push_response(uint8_t *response, size_t length) {
  bool localDebug = false;
  if (enable_verbose_debugging || localDebug) {
    pthread_mutex_lock(&g_networkMutex);
    fprintf(stderr, "PUSH RESPONSE\n");
    fprintf(stderr, "BEFORE:\n"); printQueue(g_visca_indata);
    pthread_mutex_unlock(&g_networkMutex);
  }

  uint32_t sequenceNumber = parseSequenceNumber(response, length);

  push_packet(response, length, g_visca_indata, false, sequenceNumber);
  if (enable_verbose_debugging || localDebug) {
    pthread_mutex_lock(&g_networkMutex);
    fprintf(stderr, "AFTER:\n"); printQueue(g_visca_indata);
    pthread_mutex_unlock(&g_networkMutex);
  }
}

void push_request(uint8_t *request, size_t length, bool isInquiry, uint32_t sequenceNumber) {
  bool localDebug = false;
  if (enable_verbose_debugging || localDebug) {
    pthread_mutex_lock(&g_networkMutex);
    fprintf(stderr, "BEFORE:\n"); printQueue(g_visca_outdata);
    pthread_mutex_unlock(&g_networkMutex);
  }
  push_packet(request, length, g_visca_outdata, isInquiry, sequenceNumber);
  if (enable_verbose_debugging || localDebug) {
    pthread_mutex_lock(&g_networkMutex);
    fprintf(stderr, "AFTER:\n"); printQueue(g_visca_outdata);
    pthread_mutex_unlock(&g_networkMutex);
  }
}

void push_packet(uint8_t *data, size_t length, visca_network_queue_t queue, bool isInquiry, uint32_t sequenceNumber) {
  visca_network_packet_t packet = newVISCANetworkPacket(data, length, isInquiry, sequenceNumber);

  pthread_mutex_lock(&g_networkMutex);

  packet->next = NULL;
  packet->prev = NULL;

  if (queue->tail == NULL) {
    queue->head = packet;
    queue->tail = packet;
  } else {
    queue->tail->next = packet;
    packet->prev = queue->tail;
  }

  if (enable_verbose_debugging) {
    fprintf(stderr, "Packet: %p Head: %p Tail: %p\n", packet, queue->head, queue->tail);
  }

  pthread_mutex_unlock(&g_networkMutex);
}

void add_response_handler(packetResponseHandler handler) {
  response_handler_queue_item_t item = (response_handler_queue_item_t)malloc(sizeof(*item));
  item->handlerBlock = handler;

  pthread_mutex_lock(&g_networkMutex);
  item->prev = NULL;
  item->next = g_response_handler_queue;
  if (g_response_handler_queue != NULL) {
    g_response_handler_queue->prev = item;
  }
  g_response_handler_queue = item;
  pthread_mutex_unlock(&g_networkMutex);
}

void remove_response_handler(packetResponseHandler handler) {
  pthread_mutex_lock(&g_networkMutex);

  for (response_handler_queue_item_t handlerItem = g_response_handler_queue;
       handlerItem; handlerItem = handlerItem->next) {
    if (handlerItem->handlerBlock == handler) {
      remove_response_handler_item(handlerItem);
      return;
    }
  }
  fprintf(stderr, "WARNING: Could not remove handler in remove_response_handler!\n");

  pthread_mutex_unlock(&g_networkMutex);
}

void remove_response_handler_item(response_handler_queue_item_t handler) {

  if (handler->next) {
    handler->next->prev = handler->prev;
  }
  if (handler->prev) {
    handler->prev->next = handler->next;
  } else {
    g_response_handler_queue = handler->next;
  }
  free(handler);
}

void process_incoming_packets(void) {
  int localDebug = 0;
  pthread_mutex_lock(&g_networkMutex);

  double now = getVISCATimestamp();

  if (enable_verbose_debugging || localDebug >= 2) {
    int packets = 0, handlers = 0;
    for (visca_network_packet_t packet = g_visca_indata->head ; packet ; packet = packet->next) {
      packets++;
    }
    for (response_handler_queue_item_t handler = g_response_handler_queue;
         handler; handler = handler->next) {
      handlers++;
    }

    if (packets > 0 && handlers > 0) {
      fprintf(stderr, "In process_incoming_packets (handlers = %d, packets = %d)\n", handlers, packets);
    }
  }

  for (visca_network_packet_t packet = g_visca_indata->head ; packet ; ) {
    bool handled = false;
    if (localDebug) {
      fprintf(stderr, "%lf PACKET %s\n", getVISCATimestamp(), fmtbuf(packet->data, packet->length));
    }
    for (response_handler_queue_item_t handler = g_response_handler_queue;
         handler; handler = handler->next) {
        if (handler->handlerBlock(packet)) {
            handled = true;
            if (enable_verbose_debugging) {
                fprintf(stderr, "DEQUEUE HANDLED %s\n", fmtbuf(packet->data, packet->length));
            }
            packet = deletePacketFromQueue(packet, g_visca_indata);
            remove_response_handler_item(handler);
            break;
        }
    }
    if (!handled) {
      if (localDebug) {
        fprintf(stderr, "UNHANDLED %s\n", fmtbuf(packet->data, packet->length));
      }
      if (packet->timestamp < now - 5.0) {
        if (enable_verbose_debugging) {
          fprintf(stderr, "DEQUEUE PURGE %s\n", fmtbuf(packet->data, packet->length));
        }
        packet = deletePacketFromQueue(packet, g_visca_indata);
        handled = true;
      } else {
        packet = packet->next;
      }
    }
  }
  pthread_mutex_unlock(&g_networkMutex);
}

int ackType(visca_network_packet_t packet) {
    bool localDebug = enable_verbose_debugging || false;
    int udp_offset = g_visca_use_udp ? 8 : 0;
    if (enable_verbose_debugging) {
        fprintf(stderr, "in ackType\n");
    }
    if (packet->length != 3) return -1;
    if (!packet->data) return -1;
    if (packet->data[0 + udp_offset] != 0x90 && localDebug) {
        if (localDebug) fprintf(stderr, "Unexpected ack address 0x%02x\n", packet->data[0 + udp_offset]);
        return -1;
    }
    uint8_t ack_type = packet->data[1 + udp_offset] >> 4;
    if (ack_type != 4 && ack_type != 5 && localDebug) {
        if (localDebug) fprintf(stderr, "Unexpected ack type 0x%02x\n", packet->data[1 + udp_offset]);
        return -1;
    }
    if (packet->data[2 + udp_offset] != 0xff && localDebug) {
        if (localDebug) fprintf(stderr, "Unexpected ack trailer 0x%02x\n", packet->data[2 + udp_offset]);
        return -1;
    }
    return ack_type;
}

double getVISCATimestamp(void) {
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    return curTime.tv_sec + (curTime.tv_usec / 1000000.0);
}

void *runNetworkThread(void *argIgnored) {
  while (!exit_app) {
    ssize_t localResponseLength = 0;

    pthread_mutex_lock(&g_networkMutex);
    if (g_visca_sock == -1 && g_visca_sockaddr != NULL) {
      int sock = directlyConnectToVISCAPortWithAddress(g_visca_sockaddr);
      g_visca_sock = sock;
      if (sock == -1) {
          fprintf(stderr, "VISCA failed.");
      }
      fprintf(stderr, "VISCA ready.\n");
    }
    pthread_mutex_unlock(&g_networkMutex);

    int udp_offset = g_visca_use_udp ? 8 : 0;
    bool localDebug = false;

    static uint8_t buf[65535];

    fd_set readfds, writefds;
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    if (g_visca_sock != -1) {
      FD_SET(g_visca_sock, &readfds);
      FD_SET(g_visca_sock, &writefds);
    }
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = PACKET_TIMEOUT;
    ssize_t received_length = 0;
    bool should_continue = true;

    int first_sock = select(g_visca_sock + 1, &readfds, &writefds, NULL, &tv);
    if (first_sock > 0) {
        if (FD_ISSET(g_visca_sock, &readfds)) {
            if (enable_verbose_debugging || localDebug) {
                fprintf(stderr, "Calling recvfrom\n");
            }
            received_length = recvfrom(g_visca_sock, buf, sizeof(buf),
               0, NULL, NULL);
            if (enable_verbose_debugging || localDebug) {
                fprintf(stderr, "Done (len = %llu).\n", (unsigned long long)received_length);
            }

            if (received_length > 0) {
              uint8_t *data = (uint8_t *)malloc(received_length);
              bcopy(buf, data, received_length);
              push_response(data, received_length);
            }
    
            continue;
        }
        if (FD_ISSET(g_visca_sock, &writefds)) {
            // This is seriously noisy, so require two flags to enable.
            if (enable_verbose_debugging && localDebug) {
                fprintf(stderr, "Write set\n");
            }
            pthread_mutex_lock(&g_networkMutex);
            visca_network_packet_t packet = g_visca_outdata->head;
            if (packet) {
                if (enable_verbose_debugging || localDebug) {
                    fprintf(stderr, "Sending packet\n");
                }
                if (send_visca_packet_raw(g_visca_sock, packet->data, packet->length, packet->isInquiry, packet->sequenceNumber) != -1) {
                    deletePacketFromQueue(packet, g_visca_outdata);
                }
            } else {
                // This is seriously noisy, so require two flags to enable.
                if (enable_verbose_debugging && localDebug) {
                    fprintf(stderr, "Nothing to send\n");
                }
            }
            pthread_mutex_unlock(&g_networkMutex);
        }
    } else if (first_sock == -1 && errno != EINTR) {
        // This is seriously noisy, so require two flags to enable.
        if (enable_verbose_debugging && localDebug) {
            fprintf(stderr, "Timed out waiting for ack from camera.\n");
        }
        // *len = 0;
        continue;
    } else {
        // This is seriously noisy, so require two flags to enable.
        if (enable_verbose_debugging && localDebug) { fprintf(stderr, "EINTR\n"); }
    }
    // Whether we got any new packets or not, run the response handlers just in case
    // something starts listening too late.  (Code should always add the response
    // handler before making the request, but it never hurts to run the queue again.)
    process_incoming_packets();
  }
  return NULL;
}

