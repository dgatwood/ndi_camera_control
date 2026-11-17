
#define FIELDDEF(field_name) uint8_t field_name
#define SHORTFIELDDEF(field_name, length) uint8_t field_name : length
#define MULTIFIELDDEF(field_name, length) uint8_t field_name[length]

typedef struct p2_optical_data {
#include "p2_optical_sub.h"
} p2_optical_data_t;

#undef FIELDDEF
#undef SHORTFIELDDEF
#undef MULTIFIELDDEF
#define FIELDDEF(field_name) fprintf(stderr, "%s : %d\n", #field_name, optical_data->field_name);
#define SHORTFIELDDEF(field_name, length) fprintf(stderr, "%s : %d\n", #field_name, optical_data->field_name);
#define MULTIFIELDDEF(field_name, length) { \
    uint8_t data[length+1]; \
    bcopy(optical_data->field_name, data, length); \
    data[length] = '\0'; \
    fprintf(stderr, "%s : %s\n", #field_name, data); \
}

static void print_optical_data(uint8_t *buf) {
    p2_optical_data_t *optical_data = (p2_optical_data_t *)buf;
    fprintf(stderr, "Optical data dump:\n");

#include "p2_optical_sub.h"

    fprintf(stderr, "End optical data dump\n");
}
