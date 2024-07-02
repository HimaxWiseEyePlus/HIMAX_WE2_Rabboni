/*
 * el_2nd_bl.h
 *
 *  Created on: 2023/10/24
 *      Author: Himax
 */

#ifndef APP_SCENARIO_APP_FREERTOS_SLEEP_MODE_H_
#define APP_SCENARIO_APP_FREERTOS_SLEEP_MODE_H_

/**
 * \enum SWREG_AON_MODEMOTA_UART_E
 * \brief SWREG AON Modem OTA UART
 */
typedef enum {
	SWREG_AON_MODEMOTA_UART0_FLAG = 0, /**< MODEM OTA UART0 */
	SWREG_AON_MODEMOTA_UART1_FLAG, /**< MODEM OTA UART1 */
} SWREG_AON_MODEMOTA_UART_E;

#define SWREG_AON_RETENTION_OFFSET			0x0
#define BIT_POS_REG_OTA_MODEM_UART_FLAG     18
#define BIT_SIZE_REG_OTA_MODEM_UART_FLAG    1

void hx_drv_swreg_aon_set_modemota_uart_flag(SWREG_AON_MODEMOTA_UART_E cfg);
void setCM55MTimerAlarmPMU(uint32_t timer_ms);
void app_enter_2nd_bl(uint32_t ota);

#endif  /* APP_SCENARIO_APP_FREERTOS_SLEEP_MODE_H_ */