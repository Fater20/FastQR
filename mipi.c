/* mipi.c */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "mipi.h"
//@btriet added
#include "camera_mipi.h"

static void *mipi_base;
static void *vin_base;

/* Mipi Driver Status */
static uint8_t Mipi_State = MIPI_RESET;

/* Mipi Capture Mode */
static uint8_t Mipi_CaptureMode = CONTINUOUS_MODE;

static inline uint32_t mipi_read32(int offset)
{
	return *(uint32_t *)(mipi_base + offset);
}

static inline void mipi_write32(int offset, uint32_t data)
{
	*(uint32_t *)(mipi_base + offset) = data;
}

static inline uint32_t vin_read32(int offset)
{
	return *(uint32_t *)(vin_base + offset);
}

static inline void vin_write32(int offset, uint32_t data)
{
	*(uint32_t *)(vin_base + offset) = data;
}

void mipi_set_base_addr(void *base_addr)
{
#ifdef DEBUG
	printf("mipi_set_base_addr \n");
#endif
	mipi_base = base_addr;
}

void vin_set_base_addr(void *base_addr)
{
#ifdef DEBUG
	printf("vin_set_base_addr \n");
#endif
	vin_base = base_addr;
}

/* @brief       R_MIPI_StandbyOut */
mipi_error_t R_MIPI_StandbyOut(void){
	mipi_error_t merr = MIPI_OK;
#ifdef DEBUG
	printf("R_MIPI_StandbyOut \n");
#endif
	/* Check MIPI State */
	if( ( Mipi_State != MIPI_RESET ) && ( Mipi_State != MIPI_STANDBY ) ){
		merr = MIPI_STATUS_ERR;
		printf("R_MIPI_StandbyOut MIPI_STATUS_ERR\n");
   	}
	
	if( merr == MIPI_OK ){
    	/* MIPI State Update */
    	Mipi_State = MIPI_POWOFF;
   	}

	return merr;
}

/* @brief       R_MIPI_StandbyIn*/
mipi_error_t R_MIPI_StandbyIn( void ){
    mipi_error_t merr = MIPI_OK;

#ifdef DEBUG
	printf("R_MIPI_StandbyIn \n");
#endif	
	/* Check MIPI State */
    if( Mipi_State != MIPI_POWOFF ){
        merr = MIPI_STATUS_ERR;
		printf("R_MIPI_StandbyIn MIPI_STATUS_ERR\n");
    }
	
	if( merr == MIPI_OK ){
    	/* MIPI State Update */
    	Mipi_State = MIPI_STANDBY;
		printf("R_MIPI_StandbyIn MIPI_OK => MIPI_STANDBY\n");
    }

	return merr;
}

/* @brief       R_MIPI_Open*/
mipi_error_t R_MIPI_Open(uint32_t camera_id){
    mipi_error_t merr = MIPI_OK;
	int cnt;
#ifdef DEBUG
	printf("R_MIPI_Open \n");
#endif
	/* Check MIPI State */
	if( Mipi_State != MIPI_POWOFF ){
		merr = MIPI_STATUS_ERR;
		printf("R_MIPI_Open MIPI_STATUS_ERR\n");
    	}

	if( merr == MIPI_OK ){
		#ifdef DEBUG
		printf("R_MIPI_Open Setup MIPI registers\n");
		#endif
		/* MIPI SW-Reset */
		mipi_write32(TREF_REG, 0x00000001);
		mipi_write32(SRST_REG, 0x00000001);
		for( cnt = (528000/2)*5 ; cnt > 0 ; cnt-- ){}  //5us wait
		mipi_write32(SRST_REG, 0x00000000);
	
		/*PHY Timing Register 1, 2, 3*/
		if (camera_id == CAM_IU233) {
			#ifdef DEBUG
			printf("R_MIPI_Open Setup PHYTIM for IU233\n");
			#endif
			mipi_write32(PHYTIM1_REG, 0x0000338F);
			mipi_write32(PHYTIM2_REG, 0x00031D06);
			mipi_write32(PHYTIM3_REG, 0x00001407);
		}
		else {
			#ifdef DEBUG
			printf("R_MIPI_Open Setup PHYTIM for IMX219\n");
			#endif
			mipi_write32(PHYTIM1_REG, 0x00003B5F);
			mipi_write32(PHYTIM2_REG, 0x00081E0F);
			mipi_write32(PHYTIM3_REG, 0x00001912);
		}
		/*Field Detection Control Register FLD*/
		mipi_write32(FLD_REG, 0x00000000);
		/*Checksum Control Register (CHKSUM)*/ 
		mipi_write32(CHKSUM_REG, 0x00000003);
		/*Channel Data Type Select Register (VCDT)*/
		mipi_write32(VCDT_REG, 0x011E802A);	
		/*Frame Data Type Select Register (FRDT)*/
		mipi_write32(FRDT_REG, 0x00010000);
		/*LINK Operation Control Register (LINKCNT)*/
		mipi_write32(LINKCNT_REG, 0x82000000);
		/*PHY Operation Control Register (PHYCNT)*/
		if (camera_id == CAM_IU233) {
			#ifdef DEBUG
			printf("R_MIPI_Open Setup PHYCNT_REG for IU233\n");
			#endif
			mipi_write32(PHYCNT_REG, 0x00030011);
		}
		else {
			mipi_write32(PHYCNT_REG, 0x00030013);
		}
		for( cnt = (528000/2)*25 ; cnt > 0 ; cnt-- ){}  //25us wait
		
		mipi_write32(INTSTATE_REG, 0xC0);
		/* MIPI State Update */
        Mipi_State = MIPI_STOP;
	}
	
	return merr;
}

/* @brief       R_MIPI_Close*/
mipi_error_t R_MIPI_Close( void ){
	mipi_error_t merr = MIPI_OK;
#ifdef DEBUG
	printf("R_MIPI_Close \n");
#endif
	/* Check MIPI State */
	if( ( Mipi_State != MIPI_STOP ) && ( Mipi_State != MIPI_CAPTURE ) ){
    	merr = MIPI_STATUS_ERR;
		printf("R_MIPI_Close MIPI_STATUS_ERR\n");
   	}

   	if( merr == MIPI_OK ){
		/* Interrupt Disable and Mask */
		mipi_write32(INTCLOSE_REG, 0x181FFCDD);
		vin_write32(VNIE_REG, 0x00000000);
		/* Capture Stop */
		vin_write32(VNFC_REG, 0);// Capture mode off
		vin_write32(VNMC_REG, vin_read32(VNMC_REG) & ~VNMC_ME); // VIN Stop
		if((vin_read32(VNMS_REG) & VNMS_CA) != 0)
			merr = MIPI_FATAL_ERR;
	}

	if( merr == MIPI_OK ){
		/* MIPI Reset */
		mipi_write32(PHYCNT_REG, 0x00000000);
		mipi_write32(SRST_REG, 0x00000001);
		mipi_write32(SRST_REG, 0x00000000);
	}
	return merr;
}


/* @brief       R_MIPI_Setup*/
mipi_error_t R_MIPI_Setup(void)
{
	mipi_error_t merr = MIPI_OK;
	uint8_t progressive = 1;
#ifdef DEBUG
	printf("R_MIPI_Setup debug\n");
#endif
	uint32_t vnmc, value, interrupts;
	
	/* Check MIPI State */
	if( Mipi_State != MIPI_STOP ){
    	merr = MIPI_STATUS_ERR;
		printf("R_MIPI_Setup MIPI_STATUS_ERR\n");
    }

   	 if( merr == MIPI_OK ){	
		/* VIN Initial Setting */
		vin_write32(VNMC_REG, 0x3004000A);
		/*Video n CSI2 Interface Mode Register (VnCSI_IFMD)*/
		vin_write32(VNCSI_IFMD_REG, 0x02000000);
		/*Video n Data Mode Register 2 (VnDMR2)*/
		vin_write32(VNDMR2_REG, 0x08021000);
		/*Video n Image Stride Register (VnIS)*/
		vin_write32(VNIS_REG, 0x00000190);
		/*Video n Start Line Pre-Clip Register (VnSLPrC)*/
		vin_write32(VNSLPRC_REG, 0x00000018);
		/*Video n End Line Pre-Clip Register (VnELPrC)*/
		vin_write32(VNELPRC_REG, 0x000001F7);
		/*Video n Start Pixel Pre-Clip Register (VnSPPrC)*/
		vin_write32(VNSPPRC_REG, 0x00000064);
		/*Video n End Pixel Pre-Clip Register (VnEPPrC)*/
		vin_write32(VNEPPRC_REG, 0x00000383);
		/*Video n Scaling Control Registers (VnUDS_CTRL)*/
		vin_write32(VNUDS_CTRL_REG, 0x40000000);
		/*Video n Scaling Factor Registers (VnUDS_SCALE)*/
		vin_write32(VNUDS_SCALE_REG, 0x00000000);
		/*Video n Passband Registers (VnUDS_PASS_BWIDTH)*/
		vin_write32(VNUDS_PASS_BWIDTH_REG, 0x00400040);
		/*Video n UDS Output Size Clipping Registers (VnUDS_CLIP_SIZE)*/
		vin_write32(VNUDS_CLIP_SIZE_REG, 0x00000000);

		/*Video n UV Address Offset Register (VnUVAOF)*/
		vin_write32(VNUVAOF_REG, 0x00000000);
#if 1	
		/* Ack interrupts */
		interrupts = progressive ? VNIE_FIE : VNIE_EFE;
		vin_write32(VNINTS_REG, interrupts);
		
		/* Enable interrupts */
		vin_write32(VNIE_REG, interrupts);
		
		//Start capturing
		/*Video n Data Mode Register (VnDMR)*/
		vin_write32(VNDMR_REG, 0x00000000);
		/* Enable module */
		vin_write32(VNMC_REG, 0x3004000B);
#endif
	}

	return merr;
}

/* @brief       R_MIPI_SetMode*/
mipi_error_t R_MIPI_SetMode( uint8_t CaptureMode ){
    mipi_error_t merr = MIPI_OK;
#ifdef DEBUG
	printf("R_MIPI_SetMode\n");
#endif
	/* Check MIPI State */
	if( Mipi_State != MIPI_STOP ){
		merr = MIPI_STATUS_ERR;
    	printf("R_MIPI_SetMode Wrong state\n");
    }
	if (CaptureMode > 1){
		printf("R_MIPI_SetMode Wrong mode\n");
		merr = MIPI_PARAM_ERR;
	}
	if( merr == MIPI_OK ){
		Mipi_CaptureMode = CaptureMode;
		#ifdef DEBUG
		printf("R_MIPI_SetMode %d\n", CaptureMode);
		#endif
	}
	
	return merr;
}

/* @brief       R_MIPI_SetBufferAdr*/
mipi_error_t R_MIPI_SetBufferAdr(uint8_t buffer_no, uint32_t phys_addr)
{
	mipi_error_t merr = MIPI_OK;
	uint32_t int_status;
#ifdef DEBUG
	printf("R_MIPI_SetBufferAdr \n");
#endif	

	/*Video n Interrupt Enable Register (VnIE)*/
	
	/* Check MIPI State */
    	if( ( Mipi_State != MIPI_STOP ) && ( Mipi_State != MIPI_CAPTURE ) ){
		merr = MIPI_STATUS_ERR;
		printf("R_MIPI_SetBufferAdr MIPI_STATUS_ERR\n");
    	}	
	
	if( merr == MIPI_OK ){
		/*Video n Memory Base 1,2,3 Register (VnMB1, VnMB2, VnMB3)*/
		if( buffer_no > 2 ){
			merr = MIPI_PARAM_ERR;
			printf("R_MIPI_SetBufferAdr error, not support buffer_no\n");
		} else if( ( buffer_no == 0 ) && ((uint32_t)phys_addr == 0x0) ){
			merr = MIPI_PARAM_ERR;
			printf("R_MIPI_SetBufferAdr error, phys_addr == NULL\n");
		} else if( ( ((uint32_t)phys_addr) & 0x7FUL ) != 0 ){
			merr = MIPI_PARAM_ERR;
			printf("R_MIPI_SetBufferAdr error, wrong phys_addr 0x7FUL\n");
		}
	}
	
   	if( merr == MIPI_OK ){
		if( buffer_no == 0 ){
			vin_write32(VNMB_REG(0), phys_addr);
			#ifdef DEBUG
			printf("R_MIPI_SetBufferAdr 0 to %x\n", vin_read32(VNMB_REG(0))); 
			#endif
		} else if( buffer_no == 1 ){
			vin_write32(VNMB_REG(1), phys_addr);
			#ifdef DEBUG
			printf("R_MIPI_SetBufferAdr 1 to %x\n", vin_read32(VNMB_REG(1)));
			#endif
		} else {
			vin_write32(VNMB_REG(2), phys_addr);
			#ifdef DEBUG
			printf("R_MIPI_SetBufferAdr 2 to %x\n", vin_read32(VNMB_REG(2)));
			#endif
		}
	}

	return merr;
}

/* @brief       R_MIPI_CaptureStart*/
mipi_error_t R_MIPI_CaptureStart(void){
	mipi_error_t merr = MIPI_OK;
	uint32_t int_status;

#ifdef DEBUG
	printf("R_MIPI_CaptureStart \n");
#endif		
	
	/* Check MIPI State */
    if( ( Mipi_State != MIPI_STOP ) && ( Mipi_State != MIPI_CAPTURE ) ){
        merr = MIPI_STATUS_ERR;
		printf("R_MIPI_CaptureStart MIPI_STATUS_ERR\n");
    }
    if( merr == MIPI_OK ){
		if( Mipi_CaptureMode == SINGLE_MODE ){ //Single Capture Mode 
			vin_write32(VNFC_REG, VNFC_S_FRAME);
			#ifdef DEBUG
			printf("R_MIPI_CaptureStart Single mode %x\n",vin_read32(VNFC_REG));
			#endif
		}
		else{ //Continuous capture mode
			vin_write32(VNFC_REG, VNFC_C_FRAME);
			#ifdef DEBUG
			printf("R_MIPI_CaptureStart Continuous mode %x\n",vin_read32(VNFC_REG));
			#endif
		}
	
		Mipi_State = MIPI_CAPTURE;
	}

	return merr;
}

/* @brief       R_MIPI_CaptureStop*/
mipi_error_t  R_MIPI_CaptureStop(void){
	mipi_error_t merr = MIPI_OK;
#ifdef DEBUG
	printf("R_MIPI_CaptureStop \n");
#endif	
	/* Check MIPI State */
    	if( Mipi_State != MIPI_CAPTURE ){
		merr = MIPI_STATUS_ERR;
		printf("R_MIPI_CaptureStop MIPI_STATUS_ERR\n");
   	 }
	
	/*Video n Main Control Register (VnMC)*/
	if( merr == MIPI_OK ){
		vin_write32(VNMC_REG, vin_read32(VNMC_REG) & ~VNMC_IM_MASK);    //Capture Disable
		Mipi_State = MIPI_STOP;
		printf("R_MIPI_CaptureStop setup VnMC %x\n", vin_read32(VNMC_REG));
   	}

	return merr;
}
/* @brief       R_MIPI_CaptureStop*/
unsigned int  R_MIPI_CaptureActive(void){
	mipi_error_t merr = MIPI_OK;
#ifdef DEBUG
	printf("R_MIPI_CaptureActive \n");
#endif	
	/* Check MIPI State */
    	if( Mipi_State != MIPI_CAPTURE ){
		merr = MIPI_STATUS_ERR;
		printf("R_MIPI_CaptureStop MIPI_STATUS_ERR\n");
   	 }
	
	/* Check if HW is stopped */
	if( merr == MIPI_OK ){
		#ifdef DEBUG
		printf("R_MIPI_CaptureActive %x\n", vin_read32(VNMS_REG) & VNMS_CA);
		#endif
		vin_read32(VNMS_REG) & VNMS_CA;
   	}

	return merr;
}

