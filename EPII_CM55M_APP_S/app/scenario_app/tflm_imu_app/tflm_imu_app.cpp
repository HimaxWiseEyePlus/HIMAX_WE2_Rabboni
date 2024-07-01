#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "powermode_export.h"

#define WE2_CHIP_VERSION_C		0x8538000c
#define FRAME_CHECK_DEBUG		1
#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
/* Trustzone config. */
//
/* FreeRTOS includes. */
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
#endif
#endif

#include "tflm_imu_app.h"
#include "cvapp_nycu_z_axsis.h"
#include "spi_eeprom_comm.h"
#include "atcmd_server.h"
#include "common_config.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int app_main(void) {

	std::string cmd_line;

	printf("tflm_imu_app : %s %s\r\n", __DATE__, __TIME__);

    hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);

    if ( cv_nycu_z_axsis_init(true, true, NYCU_Z_AXSIS_FLASH_ADDR) < 0 ) {
    	printf("nycu cv init fail\n");
    	return -1;
    }

	atcmd_server_init();

	loop :
		if ( get_cmd(cmd_line) == EL_OK )
		{
			exec_cmd(cmd_line);
		}
	goto loop;

	return 0;
}
