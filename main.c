/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
//#include <signal.h>
#include <termios.h>
#include <sys/time.h>

#include "lcd.h"
#include "mipi.h"
#include "camera_mipi.h"
#include "ceu.h"
#include "ov7670.h"
#include "../drp-lib/r_dk2_if.h"
#include "../drp-lib/drplib-config-api.h"
#include "../drp-lib/drp_lib/r_drp_binarization_adaptive/r_drp_binarization_adaptive.h"
#include "../drp-lib/drp_lib/r_drp_gaussian_blur/r_drp_gaussian_blur.h"

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "tag.h"
#include "client.h"

/******************************************************************************
 Memory Map
******************************************************************************/

/* See 'calculate_memory_map.sh' */

/* Video Frame Buffers */
#define CAMERA_FB_ADDR 0x80000000           /* 0x80000000  ---------------*/
					    /*             | CLUT8        */
#define CAMERA_FB_SIZE (800*480)            /* 0x8005DBFF  ---------------*/
#define OVERLAY_FB_ADDR 0x8005DC00          /* 0x8005DC00  ---------------*/
					    /*             | CLUT4        */
#define OVERLAY_FB_SIZE (800*480/2)         /* 0x8008C9FF  ---------------*/

/* Input image to DRP */
#define DRP_IN_RAM 0x8008CA00               /* 0x8008CA00  ---------------*/
					    /*             | Bayer        */
#define DRP_IN_RAM_SIZE (800*480)           /* 0x800EA5FF  ---------------*/

/* DRP work Buffers */
#define DRP_OUT_RAM 0x800EA600              /* 0x800EA600  ---------------*/
					    /*             | DRP data     */
#define DRP_OUT_RAM_SIZE (800*480)          /* 0x801481FF  ---------------*/
#define DRP_WORK_RAM 0x80148200             /* 0x80148200  ---------------*/
					    /*             | DRP data     */
#define DRP_WORK_RAM_SIZE (800*480)         /* 0x801A5DFF  ---------------*/
#define CANNY_HYST_RAM 0x801A5E00           /* 0x801A5E00  ---------------*/
					    /*             | DRP data     */
#define CANNY_HYST_RAM_SIZE (800*480)       /* 0x802039FF  ---------------*/
#define CANNY_WORK_RAM 0x80203A00           /* 0x80203A00  ---------------*/
					    /*             | DRP data     */
#define CANNY_WORK_RAM_SIZE (800*(480+3)*2) /* 0x802C04BF  ---------------*/

/* Camera Capture Buffers */
#define CEU_BUFF_ADDR 0x802C0500            /* 0x802C0500  ---------------*/
					    /*             | YCbCr422     */
#define CEU_BUFF_SIZE (800*480*2)           /* 0x8031E0FF  ---------------*/
#define MIPI_BUFF_A_ADDR 0x802C0500         /* 0x802C0500  ---------------*/
					    /*             | RAW8         */
#define MIPI_BUFF_A_SIZE (800*480)          /* 0x8031E0FF  ---------------*/
#define MIPI_BUFF_B_ADDR 0x8031E100         /* 0x8031E100  ---------------*/
					    /*             | RAW8         */
#define MIPI_BUFF_A_SIZE (800*480)          /* 0x8037BCFF  ---------------*/


/******************************************************************************
Macro definitions
******************************************************************************/
#define ON		0
#define OFF		1

#define TILE_0		(0)
#define TILE_1		(1)
#define TILE_2		(2)
#define TILE_3		(3)
#define TILE_4		(4)
#define TILE_5		(5)

#define CAP_WIDTH	800
#define CAP_HEIGHT	480

/*******************************************************************************
 Structures and Typedef definitions
*******************************************************************************/
typedef enum _demo_mode_et
{
	DRP_SOBEL_1_MODE,
	DRP_SOBEL_6_MODE,
	DRP_6_APP_MODE,
	DRP_CANNY_MODE,
	DRP_HARRIS_MODE,
	THROUGH_MODE,
	END_OF_MODE
} demo_mode_et;

char mode_strings[END_OF_MODE][30] = {
	"Apriltag",		/* DRP_SOBEL_1_MODE */
	"Sobel (1 Tile x 6)",		/* DRP_SOBEL_6_MODE */
	"6 Filters (1 Tile x 6)",	/* DRP_6_APP_MODE */
	"Canny (Dynamic Tile loading)",	/* DRP_CANNY_MODE */
	"Harris (Dynamic Tile loading)",/* DRP_HARRIS_MODE */
	"Through",			/* THROUGH_MODE */
	/* END_OF_MODE */
	};

#define MAX_LINES 12
#define LINE_INDEX "26"
struct time_stats_t
{
	char	*title;
	int	format; // 0 = ms, 1 = us
	int	val;
};

#define TOTAL_TIMES 13
int stat_count;
struct time_stats_t times[TOTAL_TIMES];

/******************************************************************************
Private global variables and functions
******************************************************************************/
demo_mode_et demo_mode = DRP_SOBEL_1_MODE;

char DRP_status=0;
int mem_fd;			/* File descriptor  to /dev/mem */


/* Video Stuff */
#define VDC_LAYER_CAMERA 0	/* Display Camera on VDC Graphics layer 0 */
#define VDC_LAYER_OVERLAY 2	/* Display Camera on VDC Graphics layer 2 */
void *layer_cam;		/* The frame buffer layer that is displaying the camera image */
void *layer_overlay;		/* The frame buffer layer that is displaying the DRP overlay graphics */

/* Buffers */
uint32_t image_buffer_addr;		/* Physical address of input image (copied from camera input) */
uint32_t drp_out_buffer_addr;		/* Physical address of DRP output buffer */
#define RAM_BASE 0x80000000
#define RAM_SIZE (4*1024*1024)
void *buffer_base = 0;			/* Userspace pointer (base) */
void *image_buffer = 0;			/* Userspace pointer to input image (copied from camera input) */
void *drp_out_buffer = 0;		/* Userspace pointer to DRP output buffer*/
void *drp_work_ram_buffer = 0;		/* Userspace pointer DRP work RAM */

/* Address for config areas in RAM */
uint32_t config_ram_sobel_phys;
uint32_t config_ram_median_phys;
uint32_t config_ram_canny_phys;
uint32_t config_ram_canny_hyst_phys;
uint32_t config_ram_harris_phys;

/* Flags */
int change_mode_flag = 1;
int mipi_input = 0;	/* get input from MIPI camera */
int ceu_input = 0;	/* get input from CEU camera */
int stats_stdout = 1;	/* Display process time stats on command line */
int stats_lcd = 1;	/* Display process time stats on screen */
int disp_cam = 1;	/* Display the camera image on the LCD */
int disp_drp = 1;	/* Display the DRP output as an overlay */
int qspi_only = 0;	/* Only load config images from QSPI, not RAM */
int button_press_flag;

#define DRP_NOT_FINISH  (0)
#define DRP_FINISH      (1)
#define DRP_TILE_EMPTY 0
#define DRP_TILE_IDLE 1
#define DRP_TILE_RUNNING 2
uint8_t drp_lib_status[R_DK2_TILE_NUM];
 int8_t drp_lib_id[R_DK2_TILE_NUM];

/* Camera Stuff */
void *map_mipi;			/* Memory mapped access to MIPI registers */
void *map_vin;			/* Memory mapped access to VIN registers */
void *mipi_buffer_a;		/* Userspace pointer to camera capture buffer for MIPI */
void *mipi_buffer_b;		/* Userspace pointer to camera capture buffer for MIPI */
int buffer_no = 0;		/* Current MIPI buffer */
int nframes = 5;		/* number of frames to get via MIPI */
void *ceu_cap_mem;		/* Userspace pointer to camera capture buffer for CEU */
void *ceu_reg;			/* Memory mapped access to CEU registers */


/*******************************************************************************
* Function Name: cb_drp_finish
* Description  : This function is a callback function called from the
*              : DRP driver at the finish of the DRP library processing.
* Arguments    : id
*              :   The ID of the DRP library that finished processing.
* Return Value : -
*******************************************************************************/
static void cb_drp_finish(uint8_t id)
{
	uint32_t tile_no;

	/* Change the operation state of the DRP library notified by the argument to finish */
	for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++)
	{
		if (drp_lib_id[tile_no] == id)
		{
			drp_lib_status[tile_no] = DRP_FINISH;
			break;
		}
	}

	return;
}

#define GR0_CLUTT 0xFCFF6000	/* GR0 CLUT Table */
#define GR2_CLUTT 0xFCFF6800	/* GR0 CLUT Table */
#define GR3_CLUTT 0xFCFF6C00	/* GR0 CLUT Table */
#define VDC6_BASE 0xFCFF7000	/* must be on a 4k page boundary */
#define SC0_SCL0_UPDATE 0x500
#define SC0_SCL0_OVR1 0x56C	/* Background color register (SC0) */
#define GR0_UPDATE 0x600
#define GR0_AB1 0x620		/* Alpha blending control register 1 (Graphics 0) */
#define GR0_AB10 0x644		/* Alpha blending control register 10 (Graphics 0) */
#define GR0_AB11 0x648		/* Alpha blending control register 11 (Graphics 0) */
#define GR0_BASE 0x64C		/* Background color control register (Graphics 0) */
#define GR0_CLUT 0x650		/* CLUT Table Control Register (Graphics 0) */
#define GR2_UPDATE 0x700
#define GR2_CLUT 0x750		/* CLUT Table Control Register (Graphics 2) */
#define GR3_UPDATE 0x780
#define GR3_CLUT_INT 0x7D0	/* CLUT Table and Interrupt Control Register (Graphics 3) */
#define ADJ0_UPDATE 0x680
#define ADJ0_ENH_TIM1 0x688	/* Enhancer timing adjustment register 1 (image quality improver 0) */
#define GR_CLT_SEL 0x00010000
int load_clut_table(void)
{
	int mem_fd;
	void *clut_base;
	void *vdc6_base;
	uint32_t value;
	uint32_t *clut_entry;
	int i;

	mem_fd = open("/dev/mem", O_RDWR);
	if(mem_fd == -1)
	{
		perror("Failed to open /dev/mem");
		return -1;
	}

	/* Map VDC6 registers */
	vdc6_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, VDC6_BASE);
	if(vdc6_base == (void *) -1)
	{
		perror("error: Could not map VDC6 registers!");
		return -1;
	}

	/* Map CLUT Table registers */
	clut_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GR0_CLUTT);
	if(clut_base == (void *) -1)
	{
		perror("error: Could not map CLUT Table registers!");
		return -1;
	}

	/* Set CLUT8 Table for displaying Camera in black and white */
	/* CLUT Table format: 32-bit entries x 256 (A R G B)
	 * We just want grayscale, so keep R,G,B all the same value.
	 * */
	clut_entry = clut_base + (0x400 * VDC_LAYER_CAMERA); 	// GRx_CLUT_TABLE
#if (VDC_LAYER_CAMERA == 0) // SW BUG/HACK/FIX
	value = 0x00800080;		// FORMAT: [XX] [Cr] [Y] [Cb]
	for (i = 0; i < 256; i++)
	{
		*clut_entry++ = value;
		value += 0x00000100;	// Just increment Y
	}
#else
	value = 0xFF000000;		// Alpha = 100%
	for (i = 0; i < 256; i++)
	{
		*clut_entry++ = value;
		value += 0x00010101;	// add 1 to R, G, B individually
	}
#endif
	value = *(volatile uint32_t *)(clut_base + (0x400 * VDC_LAYER_CAMERA));	// dummy read

	/* Toggle CLUT table select */
	/* GR0_CLUT or GR2_CLUT or GR3_CLUT_INT */
	*(volatile uint32_t *)(vdc6_base + GR0_CLUT + (0x80 * VDC_LAYER_CAMERA)) ^= GR_CLT_SEL;
	/* GR0_UPDATE or GR2_UPDATE or GR3_UPDATE */
	*(volatile uint32_t *)(vdc6_base + GR0_UPDATE + (0x80 * VDC_LAYER_CAMERA)) = 0x00000111;


	/* Set CLUT4 Table for displaying Overlay in color */
	clut_entry = clut_base + (0x400 * VDC_LAYER_OVERLAY); 	// GRx_CLUT_TABLE
	*clut_entry++ = BLACK_DATA;
	*clut_entry++ = BLUE_DATA;
	*clut_entry++ = GREEN_DATA;
	*clut_entry++ = RED_DATA;
	*clut_entry++ = WHITE_DATA;
	*clut_entry++ = CYAN_DATA;
	*clut_entry++ = YELLOW_DATA;
	*clut_entry++ = MAGENTA_DATA;
	*clut_entry++ = NAVY_DATA;
	*clut_entry++ = DARKGREEN_DATA;
	*clut_entry++ = DEEPSKYBLUE_DATA;
	*clut_entry++ = PURPLE_DATA;
	*clut_entry++ = GRAY_DATA;
	*clut_entry++ = BROWN_DATA;
	*clut_entry++ = GOLD_DATA;
	*clut_entry++ = TRANSPARENT_DATA;

	value = *(volatile uint32_t *)(clut_base + (0x400 * VDC_LAYER_OVERLAY));	// dummy read

	/* Toggle CLUT table select */
	/* GR0_CLUT or GR2_CLUT or GR3_CLUT_INT */
	*(volatile uint32_t *)(vdc6_base + GR0_CLUT + (0x80 * VDC_LAYER_OVERLAY)) ^= GR_CLT_SEL;
	/* GR0_UPDATE or GR2_UPDATE or GR3_UPDATE */
	*(volatile uint32_t *)(vdc6_base + GR0_UPDATE + (0x80 * VDC_LAYER_OVERLAY)) = 0x00000111;

	munmap(vdc6_base, 0x1000);
	munmap(clut_base, 0x1000);
	close(mem_fd);
	return 0;
}


int module_clock_enable(int mstp)
{
	int mem_fd;
	int reg;
	int bit;
	unsigned char *addr;
	void *stbcr_base;
#define STBCR_BASE 0xfcfe0000	/*mapping must always be on page boudary*/
#define STBCR3_OFFSET 0x420	/* offset to STBCR0*/

	reg = mstp /10;
	bit = mstp %10; 
	reg -= 3;		/* The MSTP values actually starts from 30, not 0 */

	mem_fd = open("/dev/mem", O_RDWR);
	if(mem_fd == -1){
		perror("Failed to open /dev/mem");
		return -1;
	}

	/* map STBCR registers and memory*/
	stbcr_base = mmap(NULL, 0x20, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, STBCR_BASE);
	if(stbcr_base == (void *) -1){
		perror("error: Could not map STBCR registers!");
		return -1;
	}

	addr = stbcr_base + STBCR3_OFFSET;	/* Add in our offset from page boundary*/
	addr+=reg * 4;

	/* Clear the mSTP bit to allow the HW module to run*/
	*addr &=  ~(1 << bit);

	munmap(stbcr_base, 0x20);
	close(mem_fd);
	return 0;
}


/******************************************************************************
* Function Name: timeval_subtract
* Description  : Subtract the ‘struct timeval’ values X and Y,
*					storing the result in RESULT
* Arguments    : - ‘struct timeval’ values X and Y
* Return Value : - 1 if the difference is negative, otherwise 0
******************************************************************************/
int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
	/* Perform the carry for the later subtraction by updating y. */
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	/* Compute the time remaining to wait.
	tv_usec is certainly positive. */
	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

/******************************************************************************
* Function Name: video_write_to_layer
* Description  :
* Arguments    :
* Return Value :
******************************************************************************/
int video_write_to_layer(char *input, char *layer)
{
#if 1
	struct timeval tv_st,tv_sp,tv_diff;

	gettimeofday(&tv_st, NULL);	/* START TIME */

	/* Output LCD buffer is in CLUT8 format (grayscale) */
	memcpy(layer, input, CAP_WIDTH*CAP_HEIGHT);

#else /* YCbCr format */
	struct timeval tv_st,tv_sp,tv_diff;
	char *y = input;
	int row, col;
	unsigned short *dst;

	gettimeofday(&tv_st, NULL);	/* START TIME */

	/* Output LCD buffer is in YCbCr format */
	/* Set all Cb and Cr values to 128 (greyscale) */
	dst = (unsigned short *)layer;
	for(row=0; row < CAP_HEIGHT; row++)
	{
		for(col = 0; col < CAP_WIDTH; col++)
		{
			*dst++ = 0x8000 | *y++;
		}
	}
#endif

	gettimeofday(&tv_sp, NULL);	/* STOP TIME */
	timeval_subtract(&tv_diff,&tv_sp,&tv_st);

	/* Record time */
	times[stat_count].val = tv_diff.tv_usec;
	times[stat_count].title = " Copy Camera->LCD (CPU):  ";
	stat_count++;

	return (int) tv_diff.tv_usec;
}

static void image_processing(uint32_t image_adr, uint32_t drp_out_adr)
{
//	r_drp_sobel_t soble_args;
	struct timeval tv1,tv2,tv3;
	int32_t ret;
	uint32_t work_area;
	static int drp_load_time;
	char buf[80];
	void *dst;
	/* Record DRP Load time */
	times[stat_count].val = drp_load_time;
	times[stat_count].title = "     DRP load from QSPI:  ";
	stat_count++;
	r_drp_binarization_adaptive_t binar_args;
	r_drp_gaussian_blur_t blur_args;
	/* Record start time */
	gettimeofday(&tv1, NULL);

	if(DRP_status)
	{	
		/* Unload all current Tile circuits  */
		/* Specifying 0 causes all loaded circuits to be unloaded. */
		R_DK2_Unload(0, drp_lib_id);

		ret = R_DK2_Load(
			(void *)(DRP_FLASH_ADDR + R_DRP_BLUR_INDEX),	// pconfig (address of image in flash)
			R_DK2_TILE_0,			// top_tiles
			R_DK2_TILE_PATTERN_1_1_1_1_1_1,	// tile_pattern
			NULL,				// pload (no callback, blocking function call)
			NULL,				// pint (no callback for R_DK2_Start)
			drp_lib_id);			// paid (ID assigned to this load)

		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Load failed (%d)\n", ret);
			exit(1);
		}
		/* Supply clock to DRP application */
		ret = R_DK2_Activate(drp_lib_id[TILE_0], 0);
		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Activate failed (%d)\n", ret);
			exit(1);
		}

		/* Prepare the parameters for Sobel */
		blur_args.src = image_adr;	/* Address of input image. */
		blur_args.dst = drp_out_adr;	/* Address of output image. */
		blur_args.width = CAP_WIDTH;	/* The horizontal size (pixels) of image. */
		blur_args.height = CAP_HEIGHT;	/* The vertical size (pixels) of image. */
		blur_args.top = 1;		/* Boundary processing(top) */
		blur_args.bottom = 1;		/* Boundary processing(bottom) */

		/* Record start time */
		gettimeofday(&tv1, NULL);

		/* Start DRP operation */
		ret = R_DK2_Start(drp_lib_id[TILE_0], &binar_args, sizeof(binar_args));
		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Start failed (%d)", ret);
			change_mode_flag = 1;	/* reload and try again */
			return;
		}

		/* Unload all current Tile circuits  */
		/* Specifying 0 causes all loaded circuits to be unloaded. */
		R_DK2_Unload(0, drp_lib_id);

		ret = R_DK2_Load(
			(void *)(DRP_FLASH_ADDR + R_DRP_BINAR_INDEX),	// pconfig (address of image in flash)
			R_DK2_TILE_0,			// top_tiles
			R_DK2_TILE_PATTERN_1_1_1_1_1_1,	// tile_pattern
			NULL,				// pload (no callback, blocking function call)
			NULL,				// pint (no callback for R_DK2_Start)
			drp_lib_id);			// paid (ID assigned to this load)

		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Load failed (%d)\n", ret);
			exit(1);
		}
		/* Supply clock to DRP application */
		ret = R_DK2_Activate(drp_lib_id[TILE_0], 0);
		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Activate failed (%d)\n", ret);
			exit(1);
		}

		/* Prepare the parameters for Sobel */
		binar_args.src = image_adr;	/* Address of input image. */
		binar_args.dst = drp_out_adr;	/* Address of output image. */
		binar_args.width = CAP_WIDTH;	/* The horizontal size (pixels) of image. */
		binar_args.height = CAP_HEIGHT;	/* The vertical size (pixels) of image. */
		binar_args.work = work_area;		
		binar_args.range = 200;		

		/* Record start time */
		gettimeofday(&tv1, NULL);

		/* Start DRP operation */
		ret = R_DK2_Start(drp_lib_id[TILE_0], &binar_args, sizeof(binar_args));
		if (ret != R_DK2_SUCCESS)
		{
			printf("R_DK2_Start failed (%d)", ret);
			change_mode_flag = 1;	/* reload and try again */
			return;
		}

	}

	tag_detect(image_buffer,layer_overlay);

	/* Record end time */
	gettimeofday(&tv2, NULL);
	timeval_subtract(&tv3,&tv2,&tv1);
	times[stat_count].val = tv3.tv_usec;
	times[stat_count].title = "              DRP Sobel:  ";
	stat_count++;
	/* Display output DRP output */
	if (disp_cam);
		video_write_to_layer(image_buffer, layer_cam);
	/* Overlay text */
	// if (stats_lcd)
	// {
	// 	//LcdClear(layer_overlay, 480);
	// 	LcdWriteString(mode_strings[DRP_SOBEL_1_MODE], 0+2, (FONTDATA_HEIGHT * 0)+ 2, GREEN, layer_overlay);
	// 	sprintf(&buf[0],"%06ld [us]", tv3.tv_usec);
	// 	LcdWriteString(&buf[0], 0+2, (FONTDATA_HEIGHT * 1)+ 2, GREEN, layer_overlay);
	// }

	return;
}



/******************************************************************************
* Function Name: main
* Description  : handler interrupt signal
* Arguments    : int signo
* Return Value : -
******************************************************************************/
//void sig_handler(int signo)
//{
//	if (signo == SIGINT)
//    {
//		/* UNMAP ALL TILES */
//		/* CLOSE ALL RESOURCES */
//		exit(1);
//	}	   
//}

int DRP_app_processing(void)
{
	int c;
	int i;
	int row, col;
	uint8_t *src, *dst;
	int timeout = 0;
	int last_mode = -1;
	int first = 1;
	int stats_lines;
	int num_lines;
	struct timeval tv_loop1,tv_loop2,tv_loop_diff;
	struct timeval tv1,tv2,tv3;
	int fps;
	int time_loop;
	int retval;
	uint32_t *V0MS;
	void *VIN_base;
	uint32_t V0MS_FBS;
	uint8_t fbs_back = 0;
	uint8_t buff_write_point = 1;
	uint32_t MIPI_buf_addr[2];
	void *MIPI_map_buf_addr[2];
	uint8_t buff_1, buff_2, buff_3;

	MIPI_buf_addr[0] = MIPI_BUFF_A_ADDR;
	MIPI_buf_addr[1] = MIPI_BUFF_B_ADDR;

	MIPI_map_buf_addr[0] = mipi_buffer_a;
	MIPI_map_buf_addr[1] = mipi_buffer_b;

	buff_1 = 0;
	buff_2 = 1;
	buff_3 = 0;
	tag_detect_init();
	if (ceu_input)
	{
		/* Capture of one frame */
		ceu_start_cap();
	}

	while(1)
	{
		gettimeofday(&tv_loop1, NULL);	/* START TIME */

		/* clear out all the time stamps */
		memset(times, 0, sizeof(times));
		stat_count = 0;

		if (mipi_input)
		{
			while (1) {
				VIN_base = mmap(NULL, 0x8, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, 0xe803f000);
				V0MS = (uint32_t *)(VIN_base + 0x4);
				V0MS_FBS = (*V0MS & 0x18) >> 3;
				if (V0MS_FBS != fbs_back && V0MS_FBS != 3) {
					munmap(VIN_base,0x8);
					break;
				}
				munmap(VIN_base,0x8);
			}
			fbs_back = V0MS_FBS;

			gettimeofday(&tv1, NULL);	/* START TIME */

			if (fbs_back == 0) {
				memcpy(image_buffer, MIPI_map_buf_addr[buff_1], CAP_WIDTH*CAP_HEIGHT);
				retval = R_MIPI_SetBufferAdr(0, MIPI_buf_addr[buff_write_point]);
				buff_1 = buff_write_point;
			}
			else if (fbs_back == 1) {
				memcpy(image_buffer, MIPI_map_buf_addr[buff_2], CAP_WIDTH*CAP_HEIGHT);
				retval = R_MIPI_SetBufferAdr(1, MIPI_buf_addr[buff_write_point]);
				buff_2 = buff_write_point;
			}
			else {
				memcpy(image_buffer, MIPI_map_buf_addr[buff_3], CAP_WIDTH*CAP_HEIGHT);
				retval = R_MIPI_SetBufferAdr(2, MIPI_buf_addr[buff_write_point]);
				buff_3 = buff_write_point;
			}

			buff_write_point ^= 1;

			gettimeofday(&tv2, NULL);	/* STOP TIME */
			timeval_subtract(&tv3,&tv2,&tv1);
			times[stat_count].val = tv3.tv_usec;
			times[stat_count].title = "  Copy MIPI image (CPU):  ";
			stat_count++;
		}

		image_processing(image_buffer_addr, drp_out_buffer_addr);

		gettimeofday(&tv_loop2, NULL);	/* STOP TIME */
		timeval_subtract(&tv_loop_diff,&tv_loop2,&tv_loop1);
		time_loop = tv_loop_diff.tv_usec;
		fps = 1000000 / time_loop;
//		printf("time: %d",time_loop);
		/* 30 FPS max */
		//if (time_loop < (1000000/30))
		//	usleep(1000000/30);


		/* Display timing stats on command line */
		if (stats_stdout)
		{
			if (first)
			{
				first = 0;
				// scroll the screen down
				for (i=0; i < MAX_LINES; i++)
					printf("\n");
			}

			/* Output title every time we change modes */
			if (last_mode != demo_mode)
			{
				last_mode = demo_mode;
				printf("\033[%dA", MAX_LINES);	// VT100: up MAX_LINES) lines

				printf("                   Mode:  \n");
				printf("                    fps:  \n");
				stats_lines = 2;

				for (i = 0; i < TOTAL_TIMES; i++)
				{
					if (times[i].val)
					{
						printf("%s\n", times[i].title);
						stats_lines++;
					}
				}
				/* Erase left over lines */
				for (i=stats_lines; i < MAX_LINES; i++)
					printf("\033[2K\n");	// erase entire line
			}

			/* VT100 cursor commands */
			// Cursor Up			<ESC>[{COUNT}A
			// Cursor Down		<ESC>[{COUNT}B
			// Cursor Forward		<ESC>[{COUNT}C
			// Cursor Backward		<ESC>[{COUNT}D
			// Erase From Current to End of Line	<ESC>[K
			// Erase Entire Line			<ESC>[2K
			#define INDEX_COL "\033[80D" "\033[" LINE_INDEX "C" 	// VT100: backup 80, forward 26(LINE_INDEX)
			#define NEXT_LINE "\033[B" 			// VT100: Down a line
			#define ERASE_AFTER "\033[K" 			// VT100: Erase till end of line (old text)
/////////////////////////////////////////////////////////////////////////////////////////
			//printf("\033[%dA", MAX_LINES);	// VT100: up 10 lines

			//printf(INDEX_COL ERASE_AFTER);
			//printf("%s", mode_strings[demo_mode]);

			//printf(NEXT_LINE INDEX_COL ERASE_AFTER);
			//printf("%d (total frame time=%dms)", fps, time_loop/1000);
//////////////////////////////////////////////////////////////////////////////////////////
			stats_lines = 2;
			for (i = 0; i < TOTAL_TIMES; i++)
			{
				if (times[i].val)
				{
					printf(NEXT_LINE INDEX_COL ERASE_AFTER);
					if (times[i].format)
						;//printf("%d us", times[i].val);
					else
						;//printf("%2d.%03d ms", times[i].val/1000, times[i].val%1000);
					stats_lines++;
				}
			}

			/* skip over blank lines */
			for (i=stats_lines; i < MAX_LINES; i++)
				printf(NEXT_LINE);

			/* leave cursor at the beginning of the next line */
			printf(NEXT_LINE "\033[80D");

			fflush(stdout);
		}
	}
}

/******************************************************************************
* Function Name: main
* Description  : main process
* Arguments    : -
* Return Value : 0 for success, -1 for error
******************************************************************************/
int main(int argc, char **argv)
{
	/**************************
	/* Initialize             */
	/**************************/

	int ret = 0, i, row, col;

	struct timeval tv1,tv2,tv3;

	int threshold = 50;

	FILE* fd_sample_image;
	int temp;

	char buf[80];
	int32_t retval;

	uint32_t camera_id;
	if( argc < 2)
	{
		printf(
		"DRP_sample: Process image by DRP\n"\
		"            and display the result on LCD.\n"\
		"DRP_sample [app]\n"\
		"[app]:\n"\
		"  -s1:   Sobelx1 mode\n"\
		"  -s6:   Sobelx6 mode\n"\
		"  -c:    Canny mode\n"\
		"  -h:    Harris mode\n"\
		"  -6a:   6 apps sample mode\n"\
		"  -nc:   Do not display live camera on LCD\n"\
		"  -nd:   Do not display DRP data overlay on LCD\n"\
		"  -qo:   Only load DRP images directly from QSPI\n"\
		"  -mipi: MIPI capture\n"\
		"  -ceu:  CEU capture (OV7670)\n"\
		);
		return -1;
	}

	/* Check options */
	i = 1; 
	for( i = 1; i < argc; i++)
	{
		if( !strcmp(argv[i],"-s1") )
		{
			demo_mode = DRP_SOBEL_1_MODE;
		}
		else if( !strcmp(argv[i],"-s6") )
		{
			demo_mode = DRP_SOBEL_6_MODE;
		}
		else if( !strcmp(argv[i],"-6a") )
		{
			demo_mode = DRP_6_APP_MODE;
		}
		else if( !strcmp(argv[i],"-c") )
		{
			demo_mode = DRP_CANNY_MODE;
		}
		else if( !strcmp(argv[i],"-h") )
		{
			demo_mode = DRP_HARRIS_MODE;
		}
		else if( !strcmp(argv[i],"-nc") )
		{
			disp_cam = 0;	/* Do not display the camera image on the LCD */
		}
		else if( !strcmp(argv[i],"-qo") )
		{
			qspi_only = 1;	/* Only load images directly from QSPI */
		}
		else if( !strcmp(argv[i],"-nd") )
		{
			disp_drp = 0;	/* Do not display the DRP output as an overlay */
		}
		else if( !strcmp(argv[i],"-m") || !strcmp(argv[i],"-mipi"))
		{
			mipi_input = 1;
		}
		else if( !strcmp(argv[i],"-ceu") )
		{
			ceu_input = 1;
		}
		else
		{
			printf("Option is not supported! (%s)\n", argv[i]);
			return -1;
		}
	}
	
	if(DRP_status)
	{
		ret = R_DK2_Initialize();
		if (ret != R_DK2_SUCCESS)
		{
		printf("R_DK2_Initialize failed\n");
		exit(1);
		}
	}

	/* Open global' handle to /dev/mem */
	mem_fd = open("/dev/mem", O_RDWR);
	if (mem_fd == -1)
	{
		perror("Cannot open /dev/mem\n");
		exit(1);
	}

#ifdef INDIVIDUAL_REMAPS
	/* Input (image) buffer to pass to DRP */
	image_buffer_addr = DRP_IN_RAM;
	image_buffer = mmap(NULL, DRP_IN_RAM_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, image_buffer_addr);

	/* DRP output buffer */
	drp_out_buffer_addr = DRP_OUT_RAM;
	drp_out_buffer = mmap(NULL, DRP_OUT_RAM_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, drp_out_buffer_addr);

	/* Intermediate work buffers for DRP image processing */
	drp_work_ram_buffer = mmap(NULL, DRP_WORK_RAM_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, DRP_WORK_RAM);


	if (	(image_buffer == MAP_FAILED) ||
		(drp_out_buffer == MAP_FAILED) ||
		(drp_work_ram_buffer == MAP_FAILED) )
	{
		perror("mmap failed\n");
		printf("image_buffer = 0x%X\n", image_buffer);
		printf("drp_out_buffer = 0x%X\n", drp_out_buffer);
		printf("drp_work_ram_buffer = 0x%X\n", drp_work_ram_buffer);
		exit(1);
	}
#else
	/* Map entire RAM area to userspace */
	buffer_base = mmap(NULL, RAM_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, RAM_BASE);
	if ( buffer_base == MAP_FAILED )
	{
		perror("mmap failed\n");
		printf("buffer_base = 0x%X\n", image_buffer);
		exit(1);
	}

	/* DRP input buffer */
	image_buffer_addr = DRP_IN_RAM;
	image_buffer = buffer_base + (DRP_IN_RAM - RAM_BASE);

	/* DRP output buffer */
	drp_out_buffer_addr = DRP_OUT_RAM;
	drp_out_buffer = buffer_base + (DRP_OUT_RAM - RAM_BASE);

	/* Intermediate work buffers for DRP image processing */
	drp_out_buffer_addr = DRP_OUT_RAM;
	drp_out_buffer = buffer_base + (DRP_OUT_RAM - RAM_BASE);
	drp_work_ram_buffer = buffer_base + (DRP_WORK_RAM - RAM_BASE);
#endif


	client_start(8000,"192.168.0.103");
	if (mipi_input)
	{
#ifdef INDIVIDUAL_REMAPS
		/* Map our capture buffers to userspace */
		mipi_buffer_a = mmap(NULL, MIPI_BUFF_A_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, MIPI_BUFF_A_ADDR);
		mipi_buffer_b = mmap(NULL, MIPI_BUFF_A_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, MIPI_BUFF_B_ADDR);
#else
		mipi_buffer_a = buffer_base + (MIPI_BUFF_A_ADDR - RAM_BASE);
		mipi_buffer_b = buffer_base + (MIPI_BUFF_B_ADDR - RAM_BASE);
#endif
		/* Map our MIPI registers to userspace */
		map_mipi = mmap(NULL, 0x26C, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, 0xe8209000);

		/* Map our VIN registers to userspace */
		map_vin = mmap(NULL, 0x30C, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, 0xe803f000);

		/* Check maps */
		if (	(mipi_buffer_a == MAP_FAILED) ||
			(mipi_buffer_b == MAP_FAILED) ||
			(map_mipi == MAP_FAILED) ||
			(map_vin == MAP_FAILED) )
		{
			perror("Could not map MIPI resources");
			goto out;
		}
		/* Pass register maps to driver */
		mipi_set_base_addr(map_mipi);
		vin_set_base_addr(map_vin);

		/* Enable clocks to VIN and MIPI */
		module_clock_enable(66);	/* VIN = MSTP66 */
		module_clock_enable(75);	/* MIPI = MSTP75 */

		/* Open I2C device for camera setting */
		MIPI_RIIC_INIT();

		/* Detect kind of camera */
		camera_id = MIPI_RIIC_CAMERA_DETECT();
		if (camera_id == CAM_IMX219){
			printf("CAM_IMX219 is connected\n");
		}
		else if (camera_id == CAM_IU233)
		{
			printf("CAM_IU233 is connected\n");
		}
		else
		{
			printf("Please connect a supported camera (Raspberry Pi Camera V2 or IU233).\n");
			goto out;
		}

		/* Camera reset */
		retval = R_MIPI_CameraReset(camera_id);
		/* Start MIPI clock supply */
		retval = R_MIPI_StandbyOut();
		/* Camera setting */
		retval = R_MIPI_CameraPowOn(camera_id);
		/* Initial setting of MIPI / VIN */
		retval = R_MIPI_Open(camera_id);
		/* Camera clock start */
		retval = R_MIPI_CameraClkStart();

		/* Set the address of our capture buffer */
		retval = R_MIPI_SetBufferAdr(0, MIPI_BUFF_A_ADDR);
		retval = R_MIPI_SetBufferAdr(1, MIPI_BUFF_B_ADDR);
		retval = R_MIPI_SetBufferAdr(2, MIPI_BUFF_A_ADDR);

		/* Set the capture mode */
		retval = R_MIPI_SetMode(1); /* 0: single mode, 1 continuous mode */
		retval = R_MIPI_Setup();

		/* Start capture */
		retval = R_MIPI_CaptureStart();
		/* Check if HW is stopped */
		while (R_MIPI_CaptureActive())
			printf("MIPI capture is ACTIVE\n");

		system("echo 3 > /proc/sys/vm/drop_caches");
	}
	else
	{
		/* Read in our test image */
		fd_sample_image = fopen("sample_image.bin", "r");
		if (!fd_sample_image ) {
			printf("Cannot open file: sample_image.bin\n");
			exit(1);
		}
		fread(image_buffer, 800*480, 1, fd_sample_image);
		fclose(fd_sample_image);
	}



	/* Configure the VDC6 layers */
	/* NOTE: We are just using 'image_buffer' here so we don't have to allocate a memory buffer*/
	if (disp_cam)
	{
		/* Configure Layer for CLUT8 to display the camera */
		sprintf(image_buffer,	"echo \"xres = 800 , yres = 480 , x_offset = 0 , y_offset = 0 , "
					"base = 0x%08X , bpp = 8 , format = CLUT8 , read_swap = swap_32_16_8 , blend = 0\" "
					" > /sys/devices/platform/soc/fcff7400.display/layer%d", CAMERA_FB_ADDR, VDC_LAYER_CAMERA);
	}
	else
	{
		/* Hide layer */
		sprintf(image_buffer,	"echo \"xres = 0 \" > /sys/devices/platform/soc/fcff7400.display/layer%d", VDC_LAYER_CAMERA);
	}
	system(image_buffer);

	if (disp_drp)
	{
		/* Configure Layer 2 CLUT4 to display the text and output of DRP */
		// NOTE: We have to use 768 instead of 800 because for CLUT4, the number of bytes for each line must be exactly divisible by 32
		sprintf(image_buffer,	"echo \"xres = 768 , yres = 480 , x_offset = 0 , y_offset = 0 , " \
					"base = 0%08X , bpp = 4 , format = CLUT4 , read_swap = swap_32_16_8 , blend = 1\" " \
					" > /sys/devices/platform/soc/fcff7400.display/layer%d", OVERLAY_FB_ADDR, VDC_LAYER_OVERLAY);
	}
	else
	{
		/* Hide layer */
		sprintf(image_buffer,	"echo \"xres = 0 \" > /sys/devices/platform/soc/fcff7400.display/layer%d", VDC_LAYER_OVERLAY);
	}
	system(image_buffer);

	/* CLUT tables for camera and overlay layers */
	load_clut_table();

	/* Setting for RGB-Chroma-key support on Layer */
	// Set the pixel color black to be transparent, so any pixel that
	// is black will automatically show the layer below (raw image).
	// This done by telling the VDC that any time it sees a RGB(0,0,0)
	// pixel in this layer to replace it with ARGB(0,0,0,0), meaning
	// the Alpha is 0% (transparent) so just let the lower layer show
	// through
	sprintf(image_buffer,	"echo layer%d = 0x00000000 to 0x00000000 > "\
				"/sys/devices/platform/soc/fcff7400.display/color_replace", VDC_LAYER_OVERLAY);
	system(image_buffer);

	/* Map our frame buffer layer to display the camera to userspace */
#ifdef INDIVIDUAL_REMAPS
	layer_cam = (unsigned char *)mmap(NULL, CAMERA_FB_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, CAMERA_FB_ADDR);
	if (layer_cam == MAP_FAILED)
	{
		perror("Could not map VDC6 camera layer");
		close(mem_fd);
		return -1;
	}
#else
	layer_cam = buffer_base + (CAMERA_FB_ADDR - RAM_BASE);
#endif
	/* Map our frame buffer layer to display the overlay to userspace */
#ifdef INDIVIDUAL_REMAPS
	layer_overlay = (unsigned char *)mmap(NULL, OVERLAY_FB_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, OVERLAY_FB_ADDR);
	if (layer_cam == MAP_FAILED)
	{
		perror("Could not map VDC6 overlay layer");
		close(mem_fd);
		return -1;
	}
#else
	layer_overlay = buffer_base + (OVERLAY_FB_ADDR - RAM_BASE);
#endif

	/* Clear our screen */
	LcdClear(layer_overlay, -1);  


	/* Jump to our main loop */
	DRP_app_processing();
	
	tag_detect_destory();

	printf("\n");
out:
	R_DK2_Unload(0, drp_lib_id);

	ov7670_close();

	/* Unmap buffers */
#ifdef INDIVIDUAL_REMAPS
	munmap(image_buffer, CAP_WIDTH*CAP_HEIGHT);
	munmap(drp_out_buffer, CAP_WIDTH*CAP_HEIGHT);
	munmap(drp_work_ram_buffer, DRP_WORK_RAM_SIZE);
	munmap(layer_cam, CAMERA_FB_SIZE);
	munmap(layer_overlay, OVERLAY_FB_SIZE);
#else
	munmap(buffer_base, 4*1024*1024);
#endif

	if (mipi_buffer_a)
		munmap(mipi_buffer_a, MIPI_BUFF_A_SIZE);
	if (mipi_buffer_b)
		munmap(mipi_buffer_b, MIPI_BUFF_A_SIZE);
	if (ceu_cap_mem)
		munmap(ceu_cap_mem, CEU_BUFF_SIZE);

	/* Unmap register */
	if (ceu_reg)
		munmap(ceu_reg, 0x100);
	if (map_mipi)
		munmap(map_mipi, 0x26C);
	if (map_vin)
		munmap(map_vin, 0x30C);

	close(mem_fd);

	return 0;
}

