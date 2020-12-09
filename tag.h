#ifndef __TAG_H
#define __TAG_H

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"
#include "common/homography.h"

typedef struct tag_info tag_info;
struct tag_info{
    int id;
    int cx;
    int cy;
    int ccx;
    int ccy;
    int p0[4][2];
    int p1[4][2];
    double d;
    double ar;
    double br;
    double cr;
    double ar_filter[3];
    double br_filter[3];
    double cr_filter[3];
};
void array_check1(double *array, double euler_angle);
void array_check2(double *array, double euler_angle);
void tag_detect_init(void);
void tag_detect(uint32_t image_adr,void *layer_overlay);
void tag_detect_destory(void);

#endif
