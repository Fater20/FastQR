#include "tag.h"
#include "lcd.h"
#include "client.h"
#include "common/homography.h"
#include "math.h"

apriltag_family_t *tf = NULL;

apriltag_detector_t *td;
char net_data[500];

tag_info tag_data[12];

double ar0,br0,cr0;
char euler_flag=1;

void array_check1(double *array, double euler_angle)
{
    double *array_flag;
    char i;
    for(i=0; array[i]&&i<3; i++);

    if(i==3)
    {
        if(fabs(euler_angle-array[2])>1.04)
        {
            array[0]=euler_angle;
            array[1]=euler_angle;
            array[2]=euler_angle;
        }
        else
        {
            array[0]=array[1];
            array[1]=array[2];
            array[2]=euler_angle;
        }
    }
    else
    {
        array[i]=euler_angle;
    }

}
void array_check2(double *array, double euler_angle)
{
    double *array_flag;
    char i;
    for(i=0; array[i]&&i<3; i++);

    if(i==3)
    {
        array[2]=euler_angle;
    }

}

void tag_detect_init(void)
{

    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 4.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;
}

void tag_detect(uint32_t image_adr,void *layer_overlay)
{
    image_u8_t im = { .width = 800,
		.height = 480,
		.stride = 800,
		.buf = (uint8_t *)image_adr
    };
        
    zarray_t *detections = apriltag_detector_detect(td, &im);
    
    for (int i = 0; i < zarray_size(detections); i++) {
       	apriltag_detection_t *det;
            
        apriltag_pose_t pose;
        apriltag_detection_info_t info;
        info.tagsize = 2;
        // info.fx = 496.1956;
        // info.fy = 496.3848;
        // info.cx = 325.7135;
        // info.cy = 237.4438;
        info.fx = 669.7198;
        info.fy = 669.5679;
        info.cx = 411.6235;
        info.cy = 239.7362;
        // info.fx = 526;
        // info.fy = 527;
        // info.cx = 400;
        // info.cy = 230;

       	zarray_get(detections, i, &det);

        info.det = det;
        estimate_tag_pose(&info, &pose);
        
        uint8_t flag=1;
        char buffer[20];

        double ar=0,br=0,cr=0;
        double quat[4];
        matrix_to_quat(pose.R, quat);
        // ar = atan2((2*(quat[0]*quat[1]+quat[2]*quat[3]))/(1-2*(quat[1]*quat[1]+quat[2]*quat[2])),quat[0]*quat[0]-quat[1]*quat[1]-quat[2]*quat[2]+quat[3]*quat[3]);
        // br = asin(2*(quat[0]*quat[2]-quat[1]*quat[3]));
        // cr = atan2((2*(quat[0]*quat[3]+quat[1]*quat[2]))/(1-2*(quat[2]*quat[2]+quat[3]*quat[3])),quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3]);

        ar = atan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),quat[0]*quat[0]-quat[1]*quat[1]-quat[2]*quat[2]+quat[3]*quat[3]);
        br = asin(2*(quat[0]*quat[2]-quat[1]*quat[3]));
        cr = atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3]);
        array_check1(tag_data[det->id].ar_filter, ar);
        array_check1(tag_data[det->id].br_filter, br);
        array_check1(tag_data[det->id].cr_filter, cr);
        if(tag_data[det->id].ar_filter[2]!=0 && tag_data[det->id].br_filter[2]!=0 && tag_data[det->id].cr_filter[2]!=0)
        {
            ar=(tag_data[det->id].ar_filter[0]+tag_data[det->id].ar_filter[1]+tag_data[det->id].ar_filter[2])/3;
            br=(tag_data[det->id].br_filter[0]+tag_data[det->id].br_filter[1]+tag_data[det->id].br_filter[2])/3;
            cr=(tag_data[det->id].cr_filter[0]+tag_data[det->id].cr_filter[1]+tag_data[det->id].cr_filter[2])/3;
        }
        
        // array_check2(tag_data[det->id].ar_filter, ar);
        // array_check2(tag_data[det->id].br_filter, br);
        // array_check2(tag_data[det->id].cr_filter, cr);
        
        // if(euler_flag)
        // {
        //     euler_flag=0;
        // }
        // else
        // {
        //     Calculate_Euler_angle(ar0, br0, cr0, &ar, &br, &cr);
        // }

        // ar0=ar;
        // br0=br;
        // cr0=cr;

        printf("ar:%lf, br:%lf, cr:%lf\n",ar,br,cr);
        double others[4][2]={0};
        double center[2]={0};
        detect_Cube_Center(&pose, info, &center[0], &center[1]);
        for (int k = 0; k < 4; k++) {
            int tcx = (k == 1 || k == 2) ? 1 : -1;
            int tcy = (k < 2) ? 1 : -1;

            //printf("tx:%lf ty:%lf tz:%lf \n",pose.t->data[0],pose.t->data[1],pose.t->data[2]);
            detect_Cube_Vertex( &pose, info, tcx , tcy, &others[k][0], &others[k][1]);
            
            //printf("others[%d][0]:%lf others[%d][1]:%lf\n", k, others[k][0], k, others[k][1]);
        }

        for(int j=0; j<4; j++)
        {
            if(det->p[j][0] < 0 || det->p[j][0] > 800 || det->p[j][1] < 0 || det->p[j][1] > 480 || others[j][0] < 0 || others[j][0] > 800 || others[j][1] < 0 || others[j][1] > 480)
                flag = 0;
            else ;
        }
        
        if(flag)
        {
            tag_data[i].id=det->id;
            tag_data[i].ccx=(int)center[0];
            tag_data[i].ccy=(int)center[1];
            tag_data[i].cx=(int)det->c[0];
            tag_data[i].cy=(int)det->c[1];
            for(int j=0; j<4; j++)
            {
                tag_data[i].p0[j][0]=(int)(det->p[j][0]);
                tag_data[i].p0[j][1]=(int)(det->p[j][1]);
                tag_data[i].p1[j][0]=(int)(others[j][0]);
                tag_data[i].p1[j][1]=(int)(others[j][1]);
            }
            tag_data[i].d= pose.t->data[2];
            tag_data[i].ar=ar;
            tag_data[i].br=br;
            tag_data[i].cr=cr;
        }
    }
    LcdClear(layer_overlay, 480);
    if(zarray_size(detections)>0)
    {
        memset(net_data,0,500);
        sprintf(net_data,"%d ",zarray_size(detections));
        for(int i=0; i < zarray_size(detections); i++)
        {
            char buffer[20];
            sprintf(net_data+strlen(net_data),"%d %d %d %lf %lf %lf %lf ", tag_data[i].id, tag_data[i].ccx, tag_data[i].ccy, tag_data[i].d, tag_data[i].ar,tag_data[i].br,tag_data[i].cr);
            
            DrawPoint(tag_data[i].cx, tag_data[i].cy, GREEN, layer_overlay);
            sprintf(buffer,"#%d d:%.2lfcm",tag_data[i].id,tag_data[i].d*5);
            //sprintf(buffer,"#%d",tag_data[i].id);
            LcdWriteString(buffer, tag_data[i].cx, tag_data[i].cy, RED, layer_overlay);
            LcdWriteLine(tag_data[i].p0[0][0],tag_data[i].p0[0][1], tag_data[i].p0[1][0],tag_data[i].p0[1][1], RED, layer_overlay);
            LcdWriteLine(tag_data[i].p0[1][0],tag_data[i].p0[1][1], tag_data[i].p0[2][0],tag_data[i].p0[2][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p0[2][0],tag_data[i].p0[2][1], tag_data[i].p0[3][0],tag_data[i].p0[3][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p0[3][0],tag_data[i].p0[3][1], tag_data[i].p0[0][0],tag_data[i].p0[0][1], BLUE, layer_overlay);
            
            LcdWriteLine(tag_data[i].p0[0][0],tag_data[i].p0[0][1], tag_data[i].p1[0][0],tag_data[i].p1[0][1], YELLOW, layer_overlay);
            LcdWriteLine(tag_data[i].p0[1][0],tag_data[i].p0[1][1], tag_data[i].p1[1][0],tag_data[i].p1[1][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p0[2][0],tag_data[i].p0[2][1], tag_data[i].p1[2][0],tag_data[i].p1[2][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p0[3][0],tag_data[i].p0[3][1], tag_data[i].p1[3][0],tag_data[i].p1[3][1], GREEN, layer_overlay);

            LcdWriteLine(tag_data[i].p1[0][0],tag_data[i].p1[0][1], tag_data[i].p1[1][0],tag_data[i].p1[1][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p1[1][0],tag_data[i].p1[1][1], tag_data[i].p1[2][0],tag_data[i].p1[2][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p1[2][0],tag_data[i].p1[2][1], tag_data[i].p1[3][0],tag_data[i].p1[3][1], GREEN, layer_overlay);
            LcdWriteLine(tag_data[i].p1[3][0],tag_data[i].p1[3][1], tag_data[i].p1[0][0],tag_data[i].p1[0][1], GREEN, layer_overlay);
        }
        printf("%s\n",net_data);
        client_send(net_data,strlen(net_data));
    }
    apriltag_detections_destroy(detections);
    // don't deallocate contents of inputs; those are the argv
}

void tag_detect_destory(void)
{
	apriltag_detector_destroy(td);
    	tag36h11_destroy(tf);
}
