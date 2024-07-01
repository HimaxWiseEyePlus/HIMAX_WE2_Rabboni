/*
 * el_2nd_bl.h
 *
 *  Created on: 2023/10/24
 *      Author: Himax
 */

#include "WE2_device.h"
#include "WE2_core.h"
#include "xprintf.h"
#include "board.h"
#include "BITOPS.h"
#include "hx_drv_swreg_aon.h"
#include "hx_drv_gpio.h"
#include "hx_drv_timer.h"
#include "powermode.h"
#include "el_2nd_bl.h"

/*******************************************************************************
 * PMU Sample Code
 ******************************************************************************/

/**
 * \brief	Set Modem OTA UART flag
 *
 * \param[in]	cfg	 Modem OTA UART flag
 * \return	void.
 */
void hx_drv_swreg_aon_set_modemota_uart_flag(SWREG_AON_MODEMOTA_UART_E cfg)
{
	uint32_t val = 0;

	val = hx_get_memory((SWREG_AON_ADDR + SWREG_AON_RETENTION_OFFSET));
	HX_BIT_SET(val, BIT_SIZE_REG_OTA_MODEM_UART_FLAG, BIT_POS_REG_OTA_MODEM_UART_FLAG,
			cfg);

	hx_set_memory((SWREG_AON_ADDR + SWREG_AON_RETENTION_OFFSET), val);
}


void setCM55MTimerAlarmPMU(uint32_t timer_ms)
{
	TIMER_CFG_T timer_cfg;

	timer_cfg.period = timer_ms;
	timer_cfg.mode = TIMER_MODE_ONESHOT;
	timer_cfg.ctrl = TIMER_CTRL_PMU;
	timer_cfg.state = TIMER_STATE_PMU;

	hx_drv_timer_cm55m_start(&timer_cfg, NULL);
}


void app_enter_2nd_bl(uint32_t ota)
{
	uint32_t boot_cnt;
	PM_PD_NOVIDPRE_CFG_T cfg;
	uint32_t freq;
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
	PM_CFG_PWR_MODE_E mode;
	SCU_PLL_FREQ_E pmuwakeup_pll_freq;
	SCU_HSCCLKDIV_E pmuwakeup_cm55m_div;
	SCU_LSCCLKDIV_E pmuwakeup_cm55s_div;

	boot_cnt = hx_get_memory(BASE_ADDR_APB_SWREG_AON_ALIAS+0x3C);
	boot_cnt++;
	hx_set_memory(BASE_ADDR_APB_SWREG_AON_ALIAS+0x3C, boot_cnt);
	xprintf("boot cnt= %d\r\n", boot_cnt);

	/*Clear PMU Wakeup Event*/
	hx_lib_pm_clear_event();

	/*Clear Wakeup related IP Interrupt*/
	hx_drv_gpio_clr_int_status(AON_GPIO0);
	hx_drv_gpio_clr_int_status(AON_GPIO1);
	hx_drv_timer_ClearIRQ(TIMER_ID_2);

	/*Get System Current Clock*/
	hx_drv_swreg_aon_get_pmuwakeup_freq(&pmuwakeup_pll_freq, &pmuwakeup_cm55m_div, &pmuwakeup_cm55s_div);
	hx_drv_swreg_aon_get_pllfreq(&freq);
	xprintf("pmuwakeup_freq_type=%d, pmuwakeup_cm55m_div=%d, pmuwakeup_cm55s_div=%d\n", pmuwakeup_pll_freq, pmuwakeup_cm55m_div, pmuwakeup_cm55s_div);
	xprintf("pmuwakeup_run_freq=%d\n", freq);

	mode = PM_MODE_PS_NOVID_PREROLLING;
	hx_lib_pm_get_defcfg_bymode(&cfg, mode);

	/*Setup bootrom clock speed when PMU Warm boot wakeup*/
	cfg.bootromspeed.bootromclkfreq = pmuwakeup_pll_freq;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = pmuwakeup_cm55m_div;
	cfg.bootromspeed.cm55s_div = pmuwakeup_cm55s_div;

	/*Setup CM55 Small can be reset*/
	cfg.cm55s_reset = SWREG_AON_PMUWAKE_CM55S_RERESET_YES;
	/*Mask RTC Interrupt for PMU*/
	cfg.pmu_rtc_mask = PM_RTC_INT_MASK_ALLMASK;
	/*Mask PA23 Interrupt for PMU*/
	cfg.pmu_pad_pa23_mask = PM_IP_INT_MASK;
	/*Mask I2CWakeup Interrupt for PMU*/
	cfg.pmu_i2cw_mask = PM_IP_INT_MASK;
	/*Mask CMP Interrupt for PMU*/
	cfg.pmu_cmp_mask = PM_IP_INT_MASK;
	/*Mask TS Interrupt for PMU*/
	cfg.pmu_ts_mask = PM_IP_INT_MASK;
	/*Mask ANTI TAMPER Interrupt for PMU*/
	cfg.pmu_anti_mask = PM_IP_INT_MASK;
	/*No Debug Dump message*/
	cfg.support_debugdump = 0;

	/*UnMask PA01 Interrupt for PMU*/
	cfg.pmu_pad_pa01_mask = PM_IP_INT_MASK_ALL_UNMASK;

	/*UnMask Timer2 Interrupt others timer interrupt are mask for PMU*/
	cfg.pmu_timer_mask = 0x1FB;

	/*Setup Memory no retention*/
	xprintf("Setup Memory no retention\n");
	cfg.tcm_retention = PM_MEM_RET_NO;			/**< CM55M TCM Retention**/
	cfg.hscsram_retention[0] = PM_MEM_RET_NO;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[1] = PM_MEM_RET_NO;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[2] = PM_MEM_RET_NO;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[3] = PM_MEM_RET_NO;	/**< HSC SRAM Retention**/
	cfg.lscsram_retention = PM_MEM_RET_NO;		/**< LSC SRAM Retention**/
	cfg.skip_bootflow.sec_mem_flag = SWREG_AON_NO_RETENTION;			/**< Skip Boot Flow**/
	cfg.skip_bootflow.first_bl_flag = SWREG_AON_NO_RETENTION; /*!< First BL Retention */
	cfg.skip_bootflow.cm55m_s_app_flag = SWREG_AON_NO_RETENTION; /*!< cm55m_s_app Retention */
	cfg.skip_bootflow.cm55m_ns_app_flag = SWREG_AON_NO_RETENTION; /*!< cm55m_ns_app Retention */
	cfg.skip_bootflow.cm55s_s_app_flag = SWREG_AON_NO_RETENTION; /*!< cm55s_s_app Retention */
	cfg.skip_bootflow.cm55s_ns_app_flag = SWREG_AON_NO_RETENTION; /*!< cm55s_ns_app Retention */
	cfg.skip_bootflow.cm55m_model_flag = SWREG_AON_NO_RETENTION; /*!< cm55m model Retention */
	cfg.skip_bootflow.cm55s_model_flag = SWREG_AON_NO_RETENTION; /*!< cm55s model Retention */
	cfg.skip_bootflow.cm55m_appcfg_flag = SWREG_AON_NO_RETENTION; /*!< cm55m appcfg Retention */
	cfg.skip_bootflow.cm55s_appcfg_flag = SWREG_AON_NO_RETENTION; /*!< cm55s appcfg Retention */
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = SWREG_AON_NO_RETENTION;/*!< cm55m_s_app RW Data Retention */
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = SWREG_AON_NO_RETENTION;/*!< cm55m_ns_app RW Data Retention */
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = SWREG_AON_NO_RETENTION;/*!< cm55s_s_app RW Data Retention */
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = SWREG_AON_NO_RETENTION;/*!< cm55s_ns_app RW Data Retention */
	cfg.skip_bootflow.secure_debug_flag = SWREG_AON_NO_RETENTION;

	/**No Pre-capture when boot up**/
	cfg.support_bootwithcap = PM_BOOTWITHCAP_NO;

	/*Not DCDC pin output*/
	cfg.pmu_dcdc_outpin = PM_CFG_DCDC_MODE_OFF;
	/** No Pre-capture when boot up**/
	cfg.ioret = PM_CFG_PD_IORET_ON;

	cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_CON;
	/*SIMO on in PD*/
	cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;

	hx_lib_pm_cfg_set(&cfg, NULL, mode);

	/* Setup CM55M Timer(Timer2) Wakeup */
	setCM55MTimerAlarmPMU(1);

	/* Use PMU lib to control HSC_CLK and LSC_CLK so set those parameter to 0 */
	memset(&hsc_cfg, 0, sizeof(SCU_PDHSC_HSCCLK_CFG_T));
	memset(&lsc_cfg, 0, sizeof(SCU_LSC_CLK_CFG_T));

	if ( ota == 1 )
	{
		hx_drv_swreg_aon_set_ota_flag(SWREG_AON_OTA_YES_FLAG);
		hx_drv_swreg_aon_set_modemota_flag(SWREG_AON_MODEMOTA_YES_FLAG);
		hx_drv_swreg_aon_set_modemota_uart_flag(SWREG_AON_MODEMOTA_UART1_FLAG);
	}

	/* Trigger to PMU mode */
	hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYPMLIB);
}
