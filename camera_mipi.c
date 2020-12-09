/* camera_mipi_imx219.c */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include "camera_mipi.h"
#include "mipi.h"

// define type of camera
#define CAM_TABLE_END 0xFFFF
#define CLIENT 0x10

int g_i2cFile;

// open the Linux device
struct cam_reg {
	uint16_t addr;
	uint8_t val;
};

struct cam_reg iu233[] = {
    /* Initialize */
    {0x0305, 0x02},  /* PREPLL DIV */
    {0x0202, 0x03},
    {0x0203, 0x20},
    {0x0205, 0xAF},
    {0x3418, 0x06},  /* 30fps 475-393*/
	{ CAM_TABLE_END, 0x00 }
};

struct cam_reg imx219[] = {
	{ 0x0114, 0x01 }, /* CSI_LANE_MODE[1:0} */
	{ 0x0128, 0x01 }, /* DPHY_CNTRL */
	{ 0x012A, 0x0C }, /* EXCK_FREQ[15:8] */
	{ 0x012B, 0x00 }, /* EXCK_FREQ[7:0] */
	{ 0x0160, 0x04 }, /* FRM_LENGTH_A[15:8] */
	{ 0x0161, 0x38 }, /* FRM_LENGTH_A[7:0] */
	{ 0x0164, 0x02 }, 
	{ 0x0165, 0xA8 }, 
	{ 0x0166, 0x0A }, 
	{ 0x0167, 0x27 }, 
	{ 0x0168, 0x02 }, 
	{ 0x0169, 0xB4 }, 
	{ 0x016A, 0x06 }, 
	{ 0x016B, 0xEB }, 
	{ 0x016C, 0x05 }, 
	{ 0x016D, 0x00 }, 
	{ 0x016E, 0x04 }, 
	{ 0x016F, 0x38 }, 
	{ 0x018C, 0x08 }, /* CSI_DATA_FORMAT_A[15:8] */
	{ 0x018D, 0x08 }, /* CSI_DATA_FORMAT_A[7:0] */
	{ 0x0172, 0x03 }, 
	{ 0x0174, 0x01 }, /* BINNING_MODE_H_A */
	{ 0x0175, 0x01 }, /* BINNING_MODE_V_A */
	{ 0x0176, 0x01 }, 
	{ 0x0177, 0x01 }, 

	{ 0x0157, 0xC0 }, 
	{ 0x0301, 0x05 }, /* VTPXCK_DIV */
	{ 0x0304, 0x02 }, /* PREPLLCK_VT_DIV[3:0] */
	{ 0x0305, 0x02 }, /* PREPLLCK_OP_DIV[3:0] */
	{ 0x0306, 0x00 }, /* PLL_VT_MPY[10:8] */
	{ 0x0307, 0x3A }, /* PLL_VT_MPY[7:0] */
	{ 0x0309, 0x08 }, /* OPPXCK_DIV[4:0] */
	{ 0x030C, 0x00 }, /* PLL_OP_MPY[10:8] */
	{ 0x030D, 0x2A }, /* PLL_OP_MPY[7:0] */

	{ CAM_TABLE_END, 0x00 }
};

void i2c_open(){
#ifdef DEBUG
	printf("i2c_open \n");
#endif	
	//TODO: decide which adapter you want to access (default: i2c-2)
	g_i2cFile = open("/dev/i2c-2", O_RDWR);
	if (g_i2cFile < 0) {
		perror("Opening I2C device node\n");
		exit(1);
	}
}

// set the I2C slave address for all subsequent I2C device transfers
void i2c_set_address(uint8_t address){
#ifdef DEBUG
	printf("i2c_set_address \n");
#endif
	if (ioctl(g_i2cFile, I2C_SLAVE_FORCE, address) < 0) {
		perror("Selecting I2C device\n");
		exit(1);
	}
}

// close the Linux device
void i2c_close(){
#ifdef DEBUG
	printf("i2c_close \n");
#endif
	close(g_i2cFile);
}

void MIPI_RIIC_INIT(){
#ifdef DEBUG
	printf("MIPI_RIIC_INIT \n");
#endif	
	i2c_open();
	i2c_set_address(CLIENT);
}
void MIPI_RIIC_CLOSE(){
	i2c_close();
}
// Write 8-bit data to 16-bit data register addr
int MIPI_RIIC_SEND(uint8_t c_addr, uint16_t reg, uint8_t val)
{
	struct i2c_rdwr_ioctl_data xfer_queue;
	struct i2c_msg msgs;
	uint8_t buf[3];
	int ret;
#ifdef DEBUG
	//printf("MIPI_RIIC_SEND \n");
#endif
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	memset(&xfer_queue, 0, sizeof(xfer_queue));
	msgs.addr = c_addr;
	msgs.len = sizeof(buf);
	msgs.flags = 0;
	msgs.buf = buf;
	
	xfer_queue.msgs = &msgs;
	xfer_queue.nmsgs = 1;

	ret = ioctl(g_i2cFile, I2C_RDWR, &xfer_queue);
	if (ret < 0)
	{
		printf("send error: 0x%04X\n", reg);
		perror("Write I2C");
		exit(1);
	}
	return 0;
}

int MIPI_RIIC_SEND_TABLE(uint8_t c_addr, struct cam_reg table[]){
	struct cam_reg *reg;
	int ret;
#ifdef DEBUG
	printf("MIPI_RIIC_SEND_TABLE \n");
#endif	
	for (reg = table; reg->addr != CAM_TABLE_END; reg++) {
		ret = MIPI_RIIC_SEND(c_addr, reg->addr, reg->val);
		if (ret < 0)
			return ret;
	}
	return 0;
}

int R_MIPI_CameraPowOn(uint32_t camera_id){
	int ret;
#ifdef DEBUG
	printf("R_MIPI_CameraPowOn \n");
#endif	
 	if(camera_id == CAM_IU233)
		ret = MIPI_RIIC_SEND_TABLE(CLIENT, iu233);
 	else
		ret = MIPI_RIIC_SEND_TABLE(CLIENT, imx219);
    if (ret < 0)
		return ret;
	return 0;
}
int R_MIPI_CameraClkStart(void){
    int ret;
#ifdef DEBUG
	printf("R_MIPI_CameraClkStart \n");
#endif		
    ret = MIPI_RIIC_SEND(CLIENT, 0x0100, 0x01);
	if (ret < 0)
		return ret;
	return 0;
}
int R_MIPI_CameraClkStop(void){
    int ret;
#ifdef DEBUG
	printf("R_MIPI_CameraClkStop \n");
#endif	
    ret = MIPI_RIIC_SEND(CLIENT, 0x0100,0x00);
	if (ret < 0)
		return ret;
	return 0;
}
int R_MIPI_CameraReset(uint32_t camera_id){
	int ret;
	int count;
#ifdef DEBUG
	printf("R_MIPI_CameraReset\n");
#endif
	if(camera_id == CAM_IMX219){
		ret = MIPI_RIIC_SEND(CLIENT, 0x0100,0x00);
		if (ret < 0)
			return ret;
	}

    ret = MIPI_RIIC_SEND(CLIENT, 0x0103,0x01);
	if (ret < 0)
		return ret;

	/* Have to wait Camera ready */
    for (count = (528 * 2000); count > 0; count--) /* CPU Clock = 528MHz, 528 clock is needed to wait 1us */
    {
        /* Do nothing 2ms(2000us) wait*/
    }
	return 0;
}
// @btriet added
// Read 8-bit data from 16-bit data register addr
int MIPI_RIIC_READ(uint8_t c_addr, uint16_t reg)
{	
	struct i2c_rdwr_ioctl_data xfer_queue;
	struct i2c_msg msgs[2];
	int ret;
	uint8_t buf[2];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msgs[0].addr = c_addr;
	msgs[0].len = sizeof(buf);
	msgs[0].flags = 0;
	msgs[0].buf = buf;

	msgs[1].addr = c_addr;
	msgs[1].len = 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = buf;

	xfer_queue.msgs = msgs;
	xfer_queue.nmsgs = 2;   
	
	ret = ioctl(g_i2cFile, I2C_RDWR, &xfer_queue);
	
	if(ret < 0)
	{
		return ret;
	}
	else 
		return buf[0];
}
int MIPI_RIIC_CAMERA_DETECT(){
	uint32_t cam_id;
	cam_id  = MIPI_RIIC_READ(0x10, 0x0000) << 8 | MIPI_RIIC_READ(0x10, 0x0001);

	if(cam_id == CAM_IMX219){
		#ifdef DEBUG
			printf("CAM_IMX219 is connected\n");
		#endif
		return CAM_IMX219;
	}
	else if(cam_id == CAM_IU233){
		#ifdef DEBUG
			printf("CAM_IU233 is connected\n");
		#endif
		return CAM_IU233;
	}
	else{
		printf(
		"Please connect a supported camera (Raspberry Pi Camera V2 or IU233).\n"\
		);
		return -1;
	}
}
