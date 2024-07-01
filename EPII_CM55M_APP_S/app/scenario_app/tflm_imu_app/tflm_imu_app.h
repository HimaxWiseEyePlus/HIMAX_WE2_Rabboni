/*
 * tflm_imu_app.h
 *
 *  Created on: Jun 12, 2024
 *      Author: 902447
 */

#ifndef APP_SCENARIO_TFLM_IMU_APP_
#define APP_SCENARIO_TFLM_IMU_APP_

#define APP_BLOCK_FUNC() do{ \
	__asm volatile("b    .");\
	}while(0)

typedef enum
{
	APP_STATE_ALLON,
}APP_STATE_E;

#ifdef __cplusplus
extern "C" {
#endif

int app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCENARIO_TFLM_IMU_APP_ */
