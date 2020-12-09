// #define DEBUG	/*uncomment this line to print out debug message*/

/* Mipi Driver Status ( for "Mipi_State" ) */
#define MIPI_RESET      (0x00)  /* First Reset */
#define MIPI_STANDBY    (0x01)  /* Clock Stopped */
#define MIPI_POWOFF     (0x02)  /* Clock Driven */
#define MIPI_STOP       (0x03)  /* Capture Stoped */
#define MIPI_CAPTURE    (0x04)  /* Capturing */

/* Mipi Capture Mode ( for "Mipi_CaptureMode" ) */
#define SINGLE_MODE     (0x00)  /* Single Capture Mode */
#define CONTINUOUS_MODE (0x01)  /* Continuous Capture Mode */

/* MIPI Error Factor */
typedef enum _mipi_error_t {
    MIPI_OK             = 0x0000,
    MIPI_FATAL_ERR      = 0x0001,
    MIPI_STATUS_ERR     = 0x0002,
    MIPI_PARAM_ERR      = 0x0003,
    MIPI_CONNECT_ERR    = 0x0004
}mipi_error_t;

/*MIPI registers*/
/* Control Timing Select */
#define TREF_REG			0x00
#define TREF_TREF			BIT(0)

/* Software Reset */
#define SRST_REG			0x04
#define SRST_SRST			BIT(0)

/* PHY Operation Control */
#define PHYCNT_REG			0x08
#define PHYCNT_SHUTDOWNZ		BIT(17)
#define PHYCNT_RSTZ			BIT(16)
#define PHYCNT_ENABLECLK		BIT(4)
#define PHYCNT_ENABLE_1			BIT(1)
#define PHYCNT_ENABLE_0			BIT(0)

/* Checksum Control */
#define CHKSUM_REG			0x0c
#define CHKSUM_ECC_EN			BIT(1)
#define CHKSUM_CRC_EN			BIT(0)

/*
 * Channel Data Type Select
 * VCDT[0-15]:  Channel 1 VCDT[16-31]:  Channel 2
 * VCDT2[0-15]: Channel 3 VCDT2[16-31]: Channel 4
 */
#define VCDT_REG			0x10
#define VCDT_VCDTN_EN			BIT(15)
#define VCDT_SEL_VC(n)			(((n) & 0x3) << 8)
#define VCDT_SEL_DTN_ON			BIT(6)
#define VCDT_SEL_DT(n)			(((n) & 0x3f) << 0)

/* Frame Data Type Select */
#define FRDT_REG			0x18

/* Field Detection Control */
#define FLD_REG				0x1c
#define FLD_FLD_NUM(n)			(((n) & 0xff) << 16)
#define FLD_FLD_EN			BIT(0)

/* Automatic Standby Control */
#define ASTBY_REG			0x20

/* Long Data Type Setting 0 */
#define LNGDT0_REG			0x28

/* Long Data Type Setting 1 */
#define LNGDT1_REG			0x2c

/* Interrupt Enable */
#define INTEN_REG			0x30

/* Interrupt Source Mask */
#define INTCLOSE_REG			0x34

/* Interrupt Status Monitor */
#define INTSTATE_REG			0x38
#define INTSTATE_INT_ULPS_START		BIT(7)
#define INTSTATE_INT_ULPS_END		BIT(6)

/* Interrupt Error Status Monitor */
#define INTERRSTATE_REG			0x3c

/* Short Packet Data */
#define SHPDAT_REG			0x40
/* Short Packet Count */
#define SHPCNT_REG			0x44

/* LINK Operation Control */
#define LINKCNT_REG			0x48
#define LINKCNT_MONITOR_EN		BIT(31)
#define LINKCNT_REG_MONI_PACT_EN	BIT(25)

/* Lane Swap */
#define LSWAP_REG			0x4c
#define LSWAP_L1SEL(n)			(((n) & 0x3) << 2)
#define LSWAP_L0SEL(n)			(((n) & 0x3) << 0)

/* PHY timing register 1*/
#define PHYTIM1_REG			0x264
#define PHYTIM1_T_INIT_SLAVE(n)		((n) & 0xFFFF)

/* PHY timing register 2*/
#define PHYTIM2_REG			0x268
#define PHYTIM2_TCLK_SETTLE(n)		(((n) & 0x3F) << 8)
#define PHYTIM2_TCLK_PREPARE(n)		((n) & 0x3F)
#define PHYTIM2_TCLK_MISS(n)		(((n) & 0x1F) << 16)

/* PHY timing register 3*/
#define PHYTIM3_REG			0x26c
#define PHYTIM3_THS_SETTLE(n)		(((n) & 0x3F) << 8)
#define PHYTIM3_THS_PREPARE(n)		((n) & 0x3F)

/*PHYDIM register*/
#define FCNTM_REG				0x170
#define PHYDIM_REG				0x180
#define PHYIM_REG				0x184
#define VINDM_REG				0x18C
#define VINSM1_REG				0x190
#define VINSM3_REG				0x198
#define PHYOM_REG				0x19C
	
/* PHY ESC Error Monitor */
#define PHEERM_REG			0x74

/* PHY Clock Lane Monitor */
#define PHCLM_REG			0x78

/* PHY Data Lane Monitor */
#define PHDLM_REG			0x7c

/*VIN registers*/
#define VNMC_REG	0x00	/* Video n Main Control Register */
#define VNMS_REG	0x04	/* Video n Module Status Register */
#define VNFC_REG	0x08	/* Video n Frame Capture Register */
#define VNSLPRC_REG	0x0C	/* Video n Start Line Pre-Clip Register */
#define VNELPRC_REG	0x10	/* Video n End Line Pre-Clip Register */
#define VNSPPRC_REG	0x14	/* Video n Start Pixel Pre-Clip Register */
#define VNEPPRC_REG	0x18	/* Video n End Pixel Pre-Clip Register */
#define VNIS_REG	0x2C	/* Video n Image Stride Register */
#define VNMB_REG(m)	(0x30 + ((m) << 2)) /* Video n Memory Base m Register */
#define VNLC		0x3C
#define VNIE_REG	0x40	/* Video n Interrupt Enable Register */
#define VNINTS_REG	0x44	/* Video n Interrupt Status Register */
#define VNSI_REG	0x48	/* Video n Scanline Interrupt Register */
#define VNDMR_REG	0x58	/* Video n Data Mode Register */
#define VNDMR2_REG	0x5C	/* Video n Data Mode Register 2 */
#define VNUVAOF_REG	0x60	/* Video n UV Address Offset Register */
#define VNCSCC_REG(m)	(0x60 + ((m) << 2)) /* Video n Memory Base m Register */
#define VNUDS_CTRL_REG		0x80	/* Scaling Control Registers */
#define VNUDS_SCALE_REG		0x84	/* Scaling Factor Register */
#define VNUDS_PASS_BWIDTH_REG	0x90	/* Passband Registers */
#define VNUDS_CLIP_SIZE_REG	0xA4	/* UDS Output Size Clipping Register */

/* Register offsets */
#define VNCSI_IFMD_REG		0x20 /* Video n CSI2 Interface Mode Register */
#define VNUTP_REG	0x100 /*Video n  lookup table pointer*/
#define VNUTD_REG	0x104 /*Video n  lookup table data register*/
#define VNYCCR(m)_REG	(0x224 + (m) << 2) /*Video RGB -> Y calculation setting register*/

/* Register bit fields for VIN */
/* Video n Main Control Register bits */
#define VNMC_CLP_NO_CLIP	(3 << 28)
#define VNMC_SCLE		(1 << 26)
#define VNMC_YCAL		(1 << 19)
#define VNMC_INF_YUV8_BT601	(1 << 16)
#define VNMC_INF_YUV10_BT601	(3 << 16)
#define VNMC_INF_RGB888		(6 << 16)
#define VNMC_INF_RAW8		(4 << 16)
#define VNMC_IM_ODD		(0 << 3)
#define VNMC_IM_ODD_EVEN	(1 << 3)
#define VNMC_IM_EVEN		(2 << 3)
#define VNMC_IM_MASK		0x18
#define VNMC_BPS		(1 << 1)
#define VNMC_ME			(1 << 0)

/* Video n Module Status Register bits */
#define VNMS_FBS_MASK		(3 << 3)
#define VNMS_FBS_SHIFT		3
#define VNMS_FS			(1 << 2)
#define VNMS_AV			(1 << 1)
#define VNMS_CA			(1 << 0)

/* Video n Frame Capture Register bits */
#define VNFC_C_FRAME		(1 << 1)
#define VNFC_S_FRAME		(1 << 0)

/* Video n Interrupt Enable Register bits */
#define VNIE_VFE		(1 << 17)
#define VNIE_FIE		(1 << 4)
#define VNIE_EFE		(1 << 1)
#define VNIE_FOE		(1 << 0)

/* Video n Interrupt Status Register bits */
#define VNINTS_FIS		(1 << 4)
#define VNINTS_EFS		(1 << 1)
#define VNINTS_FOS		(1 << 0)

/* Video n Data Mode Register bits */
#define VNDMR_EXRGB		(1 << 8)
#define VNDMR_BPSM		(1 << 4)
#define VNDMR_DTMD_YCSEP	(1 << 1)
#define VNDMR_DTMD_ARGB		(1 << 0)
#define VNDMR_DTMD_YCSEP_YCBCR420	(3 << 0)

/* Video n Data Mode Register 2 bits */
#define VNDMR2_VPS		(1 << 30)
#define VNDMR2_HPS		(1 << 29)
#define VNDMR2_FTEV		(1 << 17)
#define VNDMR2_VLV(n)		((n & 0xf) << 12)

/* Video n CSI2 Interface Mode Register */
#define VNCSI_IFMD_DES0		(1 << 25)

/* Video n UDS Control Register bits */
#define VNUDS_CTRL_AMD		(1 << 30)
#define VNUDS_CTRL_BC		(1 << 20)

#define VIN_UT_IRQ	0x01

mipi_error_t R_MIPI_SetBufferAdr(uint8_t buffer_no, uint32_t phys_addr);
void mipi_set_base_addr(void *base_addr);
void vin_set_base_addr(void *base_addr);
mipi_error_t R_MIPI_StandbyOut(void);
mipi_error_t R_MIPI_StandbyIn( void );
mipi_error_t R_MIPI_Open(uint32_t camera_id);
mipi_error_t R_MIPI_Close( void );
mipi_error_t R_MIPI_Setup(void);
mipi_error_t R_MIPI_SetMode( uint8_t CaptureMode);
mipi_error_t R_MIPI_CaptureStart(void);
unsigned int  R_MIPI_CaptureActive(void);
