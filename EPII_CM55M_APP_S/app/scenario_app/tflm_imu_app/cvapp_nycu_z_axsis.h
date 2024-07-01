/*
 * cvapp.h
 *
 *  Created on: 2024/06/12
 *      Author: 902452
 */

#ifndef SCENARIO_TFLM_IMU_APP_
#define SCENARIO_TFLM_IMU_APP_


#ifdef __cplusplus
extern "C" {
#endif

int cv_nycu_z_axsis_init(bool security_enable, bool privilege_enable, uint32_t model_addr);
int cv_nycu_z_axsis_run();
int cv_nycu_z_axsis_deinit();

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_TFLM_IMU_APP_ */
