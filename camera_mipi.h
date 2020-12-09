/* 2 supported camera */
/* Raspberri Pi Camera V2, WVGA */
#define CAM_IMX219 	0x0219
/* IU233N2 or IU233N5(1M),  VGA */
#define CAM_IU233 	0x0233

void MIPI_RIIC_INIT();
void MIPI_RIIC_CLOSE();
int R_MIPI_CameraPowOn(uint32_t camera_id);
int R_MIPI_CameraClkStart(void);
int R_MIPI_CameraClkStop(void);
int R_MIPI_CameraReset(uint32_t	camera_id);
int MIPI_RIIC_SEND(uint8_t c_addr, uint16_t reg, uint8_t val);
int MIPI_RIIC_CAMERA_DETECT();
