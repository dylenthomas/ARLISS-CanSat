#ifndef NAVIGATION_H

#include "pico/stdlib.h"
#include <math.h>

#define NAVIGATION_H

#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI / 180.0
#endif

void set_ref_pos(float ref_lon, float ref_lat, float ref_height);

static void LLH_to_ECEF(float lon, float lat, float height);
void computeNEDpos(float lon, float lat, float height, float out[3]);

#endif
