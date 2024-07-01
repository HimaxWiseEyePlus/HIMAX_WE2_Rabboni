#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "powermode_export.h"




// #define WATCH_DOG_TIMEOUT_TH	(20000) //ms
#define WATCH_DOG_TIMEOUT_TH	(500) //ms

#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
/* Trustzone config. */
//
/* FreeRTOS includes. */
//#include "secure_port_macros.h"
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
//#include "veneer_table.h"
//
#endif
#endif

#include "WE2_device.h"

#include "spi_master_protocol.h"
#include "hx_drv_spi.h"
#include "spi_eeprom_comm.h"
#include "board.h"
#include "xprintf.h"
#include "tflm_nycu_z_axsis.h"
#include "board.h"
#include "WE2_core.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#ifdef IP_sensorctrl
#include "hx_drv_sensorctrl.h"
#endif
#ifdef IP_xdma
#include "hx_drv_xdma.h"
#include "sensor_dp_lib.h"
#endif
#ifdef IP_cdm
#include "hx_drv_cdm.h"
#endif
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif
#include "hx_drv_pmu_export.h"
#include "hx_drv_pmu.h"
#include "powermode.h"
//#include "dp_task.h"
#include "BITOPS.h"

#include "common_config.h"
#include "cisdp_sensor.h"
#include "event_handler.h"
#include "cvapp_nycu_z_axsis.h"
#include "memory_manage.h"
#include "hx_drv_watchdog.h"

#ifdef EPII_FPGA
#define DBG_APP_LOG             (1)
#else
#define DBG_APP_LOG             (0)
#endif
#if DBG_APP_LOG
    #define dbg_app_log(fmt, ...)       xprintf(fmt, ##__VA_ARGS__)
#else
    #define dbg_app_log(fmt, ...)
#endif

#define TOTAL_STEP_TICK 1
#define TOTAL_STEP_TICK_DBG_LOG 0

#if TOTAL_STEP_TICK
#define CPU_CLK	0xffffff+1
#endif

#define GROVE_VISION_AI_II

static uint8_t 	g_xdma_abnormal, g_md_detect, g_cdm_fifoerror, g_wdt1_timeout, g_wdt2_timeout,g_wdt3_timeout;
static uint8_t 	g_hxautoi2c_error, g_inp1bitparer_abnormal;
static uint32_t g_dp_event;
static uint8_t 	g_frame_ready;
static uint32_t g_cur_jpegenc_frame;
static uint8_t 	g_time;
static uint8_t g_spi_master_initial_status;
static uint32_t g_use_case;
/*volatile*/ uint32_t jpeg_addr, jpeg_sz;

static uint32_t g_trans_type;
static uint32_t judge_case_data;
void app_start_state(APP_STATE_E state);
void model_change(void);
void pinmux_init();

#ifdef GROVE_VISION_AI_II
/* Init SPI master pin mux (share with SDIO) */
void spi_m_pinmux_cfg(SCU_PINMUX_CFG_T *pinmux_cfg)
{
	pinmux_cfg->pin_pb2 = SCU_PB2_PINMUX_SPI_M_DO_1;        /*!< pin PB2*/
	pinmux_cfg->pin_pb3 = SCU_PB3_PINMUX_SPI_M_DI_1;        /*!< pin PB3*/
	pinmux_cfg->pin_pb4 = SCU_PB4_PINMUX_SPI_M_SCLK_1;      /*!< pin PB4*/
	pinmux_cfg->pin_pb11 = SCU_PB11_PINMUX_SPI_M_CS;        /*!< pin PB11*/
}
#else
/* Init SPI master pin mux (share with SDIO) */
void spi_m_pinmux_cfg(SCU_PINMUX_CFG_T *pinmux_cfg)
{
	pinmux_cfg->pin_pb2 = SCU_PB2_PINMUX_SPI_M_DO_1;        /*!< pin PB2*/
	pinmux_cfg->pin_pb3 = SCU_PB3_PINMUX_SPI_M_DI_1;        /*!< pin PB3*/
	pinmux_cfg->pin_pb4 = SCU_PB4_PINMUX_SPI_M_SCLK_1;      /*!< pin PB4*/
	pinmux_cfg->pin_pb5 = SCU_PB5_PINMUX_SPI_M_CS_1;        /*!< pin PB5*/
}
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void pinmux_init()
{
	SCU_PINMUX_CFG_T pinmux_cfg;

	hx_drv_scu_get_all_pinmux_cfg(&pinmux_cfg);

	/* Init SPI master pin mux (share with SDIO) */
	spi_m_pinmux_cfg(&pinmux_cfg);

	hx_drv_scu_set_all_pinmux_cfg(&pinmux_cfg, 1);
}


static void dp_var_int()
{
	g_xdma_abnormal = 0;
	g_md_detect = 0;
	g_cdm_fifoerror = 0;
	g_wdt1_timeout = 0;
	g_wdt2_timeout = 0;
	g_wdt3_timeout = 0;
	g_inp1bitparer_abnormal = 0;
	g_dp_event = 0;
	g_frame_ready = 0;
	g_time = 0;
	g_cur_jpegenc_frame = 0;
	g_hxautoi2c_error = 0;
	g_spi_master_initial_status = 0;
}

void WDG_Reset_ISR_CB (uint32_t event)
{
	uint32_t read_data;
    read_data = hx_drv_watchdog_value(WATCHDOG_ID_0);
    xprintf ("read_data=%d not reset\n", read_data);
	xprintf("CLI_WDG_Reset_ISR_CB event=%d\n", event);
	//hx_drv_watchdog_stop();
}

void SetAlarmPMU() {
	uint32_t id;
	TIMER_CFG_T timer_cfg;
	//TIMER_ERROR_E ret;

#ifdef EPII_FPGA
	timer_cfg.period = 1000;//30000;
#else
	timer_cfg.period = 1000;//30000;
#endif
	timer_cfg.mode = TIMER_MODE_ONESHOT;
	timer_cfg.ctrl = TIMER_CTRL_PMU;
	timer_cfg.state = TIMER_STATE_PMU;
	id = 1;

	//ret = hx_drv_timer_hw_start(id, &timer_cfg, NULL);
	hx_drv_timer_hw_start(id, &timer_cfg, NULL);
}

void SetPSPDNoVid()
{
	PM_PD_NOVIDPRE_CFG_T cfg;
	uint8_t speed,reset, precap, nframeend_ctrl, trigger, retention;
	uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
	uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
	uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask;
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
	PM_CFG_PWR_MODE_E mode;


	speed = SCU_PLL_FREQ_ENABLE;

	reset = 0;
	nframeend_ctrl = 0;

	retention = 0;

	precap = 0;
	pmu_pad_pa01_mask = 0;
	pmu_rtc_mask = 0;
	pmu_pad_pa23_mask = 0;
	pmu_i2cw_mask = 0;
	pmu_timer_mask = 0;
	pmu_cmp_mask = 0;
	pmu_ts_mask = 0;
	trigger = 1;
	support_debugdump = 0;
	dcdcpin = 0;

	freq = 400000000;
	cm55mdiv = SCU_HSCCLKDIV_1;
	cm55sdiv = SCU_LSCCLKDIV_4;

	pmu_anti_mask = 0;
	dbg_app_log("speed=%d,reset=%d,nframeend_ctrl=%d,retention=%d,precap=%d\n", speed,reset,nframeend_ctrl,retention,precap);
	dbg_app_log("ag_mask=0x%x,rtc_mask=0x%x,sb_mask=0x%x,i2cw_mask=0x%x,timer_mask=0x%x,cmp_mask=0x%x,ts_mask=0x%x\n", pmu_pad_pa01_mask,pmu_rtc_mask,pmu_pad_pa23_mask,pmu_i2cw_mask,pmu_timer_mask,pmu_cmp_mask,pmu_ts_mask);
	dbg_app_log("trigger=%d,debug=%d, reset=%d\n", trigger,support_debugdump, reset);
	dbg_app_log("dcdcpin=%d, pmu_anti_mask=0x%x\n", dcdcpin, pmu_anti_mask);
	dbg_app_log("freq=%d, cm55mdiv=%d,cm55sdiv=%d\n", freq, cm55mdiv, cm55sdiv);

	mode = PM_MODE_PS_NOVID_PREROLLING;
	hx_lib_pm_get_defcfg_bymode(&cfg, mode);

	cfg.bootromspeed.bootromclkfreq = speed;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = cm55mdiv;
	cfg.bootromspeed.cm55s_div = cm55sdiv;


	cfg.cm55s_reset = reset;
	cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
	cfg.pmu_rtc_mask = pmu_rtc_mask;
	cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;			/**< PMU SB GPIO Interrupt Mask **/
	cfg.pmu_i2cw_mask = pmu_i2cw_mask;			/**< PMU I2C Wakeup Interrupt Mask **/
	cfg.pmu_timer_mask = pmu_timer_mask;			/**< PMU Timer0~5 Wakeup Interrupt Mask  **/
	cfg.pmu_cmp_mask = pmu_cmp_mask;			/**< PMU CMP Wakeup Interrupt Mask  **/
	cfg.pmu_ts_mask = pmu_ts_mask;			/**< PMU TS Wakeup Interrupt Mask  **/
	cfg.pmu_anti_mask = pmu_anti_mask;
	cfg.support_debugdump = support_debugdump;

	cfg.nframeend_ctrl = nframeend_ctrl;	/**< NFrame Control **/

	cfg.tcm_retention = retention;			/**< CM55M TCM Retention**/
	cfg.hscsram_retention[0] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[1] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[2] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[3] = retention;	/**< HSC SRAM Retention**/
	cfg.lscsram_retention = retention;		/**< LSC SRAM Retention**/
	cfg.skip_bootflow.sec_mem_flag = retention;			/**< Skip Boot Flow**/
	cfg.skip_bootflow.first_bl_flag = retention; /*!< First BL Retention */
	cfg.skip_bootflow.cm55m_s_app_flag = retention; /*!< cm55m_s_app Retention */
	cfg.skip_bootflow.cm55m_ns_app_flag = retention; /*!< cm55m_ns_app Retention */
	cfg.skip_bootflow.cm55s_s_app_flag = retention; /*!< cm55s_s_app Retention */
	cfg.skip_bootflow.cm55s_ns_app_flag = retention; /*!< cm55s_ns_app Retention */
	cfg.skip_bootflow.cm55m_model_flag = retention; /*!< cm55m model Retention */
	cfg.skip_bootflow.cm55s_model_flag = retention; /*!< cm55s model Retention */
	cfg.skip_bootflow.cm55m_appcfg_flag = retention; /*!< cm55m appcfg Retention */
	cfg.skip_bootflow.cm55s_appcfg_flag = retention; /*!< cm55s appcfg Retention */
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;/*!< cm55m_s_app RW Data Retention */
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;/*!< cm55m_ns_app RW Data Retention */
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;/*!< cm55s_s_app RW Data Retention */
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;/*!< cm55s_ns_app RW Data Retention */
	cfg.skip_bootflow.secure_debug_flag = retention;
	cfg.support_bootwithcap = precap;		/**< Support capture when boot up**/
	cfg.pmu_dcdc_outpin = dcdcpin;
	cfg.ioret = PM_CFG_PD_IORET_ON;
	cfg.mipi_lane_en = PMU_MIPI_LANE_ALL_DISABLE;
	cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_SHIFT;
	cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;

	hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_NOVID_PREROLLING);

	SetAlarmPMU();

	hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_XTAL24M;
	hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
	hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
	hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
	hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
	lsc_cfg.lscclksrc = SCU_LSCCLKSRC_XTAL24M;
	lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;

	if(trigger == 1)
	{
		hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
	}

}

void SetPSPDNoVid_24M()
{
	PM_PD_NOVIDPRE_CFG_T cfg;
	uint8_t speed,reset, precap, nframeend_ctrl, trigger, retention;
	uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
	uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
	uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask;
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
	PM_CFG_PWR_MODE_E mode;


	speed = SCU_PLL_FREQ_DISABLE;

	reset = 1;
	nframeend_ctrl = 0;

	retention = 0;

	precap = 0;
	pmu_pad_pa01_mask = 0;
	pmu_rtc_mask = 0;
	pmu_pad_pa23_mask = 0;
	pmu_i2cw_mask = 0;
	pmu_timer_mask = 0;
	pmu_cmp_mask = 0;
	pmu_ts_mask = 0;
	trigger = 1;
	support_debugdump = 0;
	dcdcpin = 0;

	freq = 0;
	cm55mdiv = SCU_HSCCLKDIV_1;
	cm55sdiv = SCU_LSCCLKDIV_1;

	pmu_anti_mask = 0;
	dbg_app_log("speed=%d,reset=%d,nframeend_ctrl=%d,retention=%d,precap=%d\n", speed,reset,nframeend_ctrl,retention,precap);
	dbg_app_log("ag_mask=0x%x,rtc_mask=0x%x,sb_mask=0x%x,i2cw_mask=0x%x,timer_mask=0x%x,cmp_mask=0x%x,ts_mask=0x%x\n", pmu_pad_pa01_mask,pmu_rtc_mask,pmu_pad_pa23_mask,pmu_i2cw_mask,pmu_timer_mask,pmu_cmp_mask,pmu_ts_mask);
	dbg_app_log("trigger=%d,debug=%d, reset=%d\n", trigger,support_debugdump, reset);
	dbg_app_log("dcdcpin=%d, pmu_anti_mask=0x%x\n", dcdcpin, pmu_anti_mask);
	dbg_app_log("freq=%d, cm55mdiv=%d,cm55sdiv=%d\n", freq, cm55mdiv, cm55sdiv);

	mode = PM_MODE_PS_NOVID_PREROLLING;
	hx_lib_pm_get_defcfg_bymode(&cfg, mode);

	cfg.bootromspeed.bootromclkfreq = speed;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = cm55mdiv;
	cfg.bootromspeed.cm55s_div = cm55sdiv;


	cfg.cm55s_reset = reset;
	cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
	cfg.pmu_rtc_mask = pmu_rtc_mask;
	cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;			/**< PMU SB GPIO Interrupt Mask **/
	cfg.pmu_i2cw_mask = pmu_i2cw_mask;			/**< PMU I2C Wakeup Interrupt Mask **/
	cfg.pmu_timer_mask = pmu_timer_mask;			/**< PMU Timer0~5 Wakeup Interrupt Mask  **/
	cfg.pmu_cmp_mask = pmu_cmp_mask;			/**< PMU CMP Wakeup Interrupt Mask  **/
	cfg.pmu_ts_mask = pmu_ts_mask;			/**< PMU TS Wakeup Interrupt Mask  **/
	cfg.pmu_anti_mask = pmu_anti_mask;
	cfg.support_debugdump = support_debugdump;

	cfg.nframeend_ctrl = nframeend_ctrl;	/**< NFrame Control **/

	cfg.tcm_retention = retention;			/**< CM55M TCM Retention**/
	cfg.hscsram_retention[0] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[1] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[2] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[3] = retention;	/**< HSC SRAM Retention**/
	cfg.lscsram_retention = retention;		/**< LSC SRAM Retention**/
	cfg.skip_bootflow.sec_mem_flag = retention;			/**< Skip Boot Flow**/
	cfg.skip_bootflow.first_bl_flag = retention; /*!< First BL Retention */
	cfg.skip_bootflow.cm55m_s_app_flag = retention; /*!< cm55m_s_app Retention */
	cfg.skip_bootflow.cm55m_ns_app_flag = retention; /*!< cm55m_ns_app Retention */
	cfg.skip_bootflow.cm55s_s_app_flag = retention; /*!< cm55s_s_app Retention */
	cfg.skip_bootflow.cm55s_ns_app_flag = retention; /*!< cm55s_ns_app Retention */
	cfg.skip_bootflow.cm55m_model_flag = retention; /*!< cm55m model Retention */
	cfg.skip_bootflow.cm55s_model_flag = retention; /*!< cm55s model Retention */
	cfg.skip_bootflow.cm55m_appcfg_flag = retention; /*!< cm55m appcfg Retention */
	cfg.skip_bootflow.cm55s_appcfg_flag = retention; /*!< cm55s appcfg Retention */
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;/*!< cm55m_s_app RW Data Retention */
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;/*!< cm55m_ns_app RW Data Retention */
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;/*!< cm55s_s_app RW Data Retention */
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;/*!< cm55s_ns_app RW Data Retention */
	cfg.skip_bootflow.secure_debug_flag = retention;
	cfg.support_bootwithcap = precap;		/**< Support capture when boot up**/
	cfg.pmu_dcdc_outpin = dcdcpin;
	cfg.ioret = PM_CFG_PD_IORET_ON;
	cfg.mipi_lane_en = PMU_MIPI_LANE_ALL_DISABLE;
	cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_SHIFT;
	cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;

	hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_NOVID_PREROLLING);

	SetAlarmPMU();

	hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_XTAL24M;
	hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
	hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
	hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
	hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
	lsc_cfg.lscclksrc = SCU_LSCCLKSRC_XTAL24M;
	lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;

	if(trigger == 1)
	{
		hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
	}

}

void SetPSAudVidonly_24M() {
	PM_PD_VIDAUDPRE_CFG_T cfg;
	uint8_t speed,reset, trigger, retention;
	uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
	uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
	uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask, pmu_senint_mask;
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;


	speed = SCU_PLL_FREQ_DISABLE;
	reset = 1;
	retention = 0;

	pmu_pad_pa01_mask = 0;
	pmu_rtc_mask = 0;
	pmu_pad_pa23_mask = 0;
	pmu_i2cw_mask = 0;
	pmu_timer_mask = 0; //TIMER876543210, TIMER MASK1 = 0x2
	pmu_cmp_mask = 0;
	pmu_ts_mask = 0;
	trigger = 1;
	support_debugdump = 0;
	dcdcpin = 0;
	pmu_senint_mask = 0;

	freq = 24000000;
	cm55mdiv = SCU_HSCCLKDIV_1;
	cm55sdiv = SCU_LSCCLKDIV_1;

	pmu_anti_mask = 0;

	cfg.bootromspeed.bootromclkfreq = speed;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = cm55mdiv;
	cfg.bootromspeed.cm55s_div = cm55sdiv;

#ifdef EPII_FPGA
	cfg.sensor_timer = 900;//29900;	 /**< Sensor Timer **/
	cfg.wdt_timer = 1000;	 /**< WDT Timer **/
#else
	cfg.sensor_timer = 900;//29900;	 /**< Sensor Timer **/
	cfg.wdt_timer = 1000;	 /**< WDT Timer **/
#endif
	cfg.nframeend_ctrl = PMU_NFRAMEEND_CTRL_I2C;
	cfg.cm55s_reset = reset;
	cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
	cfg.pmu_rtc_mask = pmu_rtc_mask;
	cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;			/**< PMU SB GPIO Interrupt Mask **/
	cfg.pmu_i2cw_mask = pmu_i2cw_mask;			/**< PMU I2C Wakeup Interrupt Mask **/
	cfg.pmu_timer_mask = pmu_timer_mask;			/**< PMU Timer0~5 Wakeup Interrupt Mask  **/
	cfg.pmu_cmp_mask = pmu_cmp_mask;			/**< PMU CMP Wakeup Interrupt Mask  **/
	cfg.pmu_ts_mask = pmu_ts_mask;			/**< PMU TS Wakeup Interrupt Mask  **/
	cfg.pmu_anti_mask = pmu_anti_mask;
	cfg.pmu_mipii2c_noack_mask = 0;
	cfg.pmu_senint_mask = pmu_senint_mask;		/**< PMU SENSOR Interrupt Mask **/
	cfg.support_debugdump = support_debugdump;

	cfg.tcm_retention = retention;			/**< CM55M TCM Retention**/
	cfg.hscsram_retention[0] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[1] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[2] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[3] = retention;	/**< HSC SRAM Retention**/
	cfg.lscsram_retention = retention;		//cfg.lscsram_retention = retention;		/**< LSC SRAM Retention**/
	cfg.skip_bootflow.sec_mem_flag = retention;			/**< Skip Boot Flow**/
	cfg.skip_bootflow.first_bl_flag = retention; /*!< First BL Retention */
	cfg.skip_bootflow.cm55m_s_app_flag = retention; /*!< cm55m_s_app Retention */
	cfg.skip_bootflow.cm55m_ns_app_flag = retention; /*!< cm55m_ns_app Retention */
	cfg.skip_bootflow.cm55s_s_app_flag = retention;//cfg.skip_bootflow.cm55s_s_app_flag = retention; /*!< cm55s_s_app Retention */
	cfg.skip_bootflow.cm55s_ns_app_flag = retention;//cfg.skip_bootflow.cm55s_ns_app_flag = retention; /*!< cm55s_ns_app Retention */
	cfg.skip_bootflow.cm55m_model_flag = retention; /*!< cm55m model Retention */
	cfg.skip_bootflow.cm55s_model_flag = retention;//cfg.skip_bootflow.cm55s_model_flag = retention; /*!< cm55s model Retention */
	cfg.skip_bootflow.cm55m_appcfg_flag = retention; /*!< cm55m appcfg Retention */
	cfg.skip_bootflow.cm55s_appcfg_flag = retention;//cfg.skip_bootflow.cm55s_appcfg_flag = retention; /*!< cm55s appcfg Retention */
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;/*!< cm55m_s_app RW Data Retention */
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;/*!< cm55m_ns_app RW Data Retention */
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;//cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;/*!< cm55s_s_app RW Data Retention */
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;//cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;/*!< cm55s_ns_app RW Data Retention */

	cfg.fast_vpr = 0;
	cfg.pmu_dcdc_outpin = dcdcpin;

	hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_VID_ONLY_PREROLLING);

	SetAlarmPMU();

	hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_XTAL24M;
	hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
	hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
	hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
	hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
	lsc_cfg.lscclksrc = SCU_LSCCLKSRC_XTAL24M;
	lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;


	if(trigger == 1)
	{
		hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
	}
}
void SetPSAudVidonly()
{
	PM_PD_VIDAUDPRE_CFG_T cfg;
	uint8_t speed,reset, trigger, retention;
	uint32_t pmu_pad_pa01_mask, pmu_rtc_mask, support_debugdump;
	uint32_t pmu_pad_pa23_mask, pmu_i2cw_mask, pmu_timer_mask, pmu_cmp_mask, pmu_ts_mask;
	uint32_t dcdcpin, freq, cm55mdiv, cm55sdiv, pmu_anti_mask, pmu_senint_mask;
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;


#ifdef SUPPORT_WARMBOOT_DISPLL
	speed = SCU_PLL_FREQ_DISABLE;
#else
	speed = SCU_PLL_FREQ_ENABLE;
#endif
#ifdef SUPPORT_CM55S_RESET
	reset = 1;
#else
	reset = 0;
#endif
#ifdef SUPPORT_MEM_RETENTION
	retention = 1;
#else
	retention = 0;
#endif

	pmu_pad_pa01_mask = 0;
	pmu_rtc_mask = 0;
	pmu_pad_pa23_mask = 0;
	pmu_i2cw_mask = 0;
	pmu_timer_mask = ~(0x1<<1); //TIMER876543210, TIMER MASK1 = 0x2
	pmu_cmp_mask = 0;
	pmu_ts_mask = 0;
	trigger = 1;
	support_debugdump = 0;
	dcdcpin = 0;
	pmu_senint_mask = 0;

	freq = 400000000;
	cm55mdiv = SCU_HSCCLKDIV_1;
	cm55sdiv = SCU_LSCCLKDIV_4;

	pmu_anti_mask = 0;

	cfg.bootromspeed.bootromclkfreq = speed;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = cm55mdiv;
	cfg.bootromspeed.cm55s_div = cm55sdiv;

#ifdef EPII_FPGA
	cfg.sensor_timer = 900;//29900;	 /**< Sensor Timer **/
	cfg.wdt_timer = 1000;	 /**< WDT Timer **/
#else
	cfg.sensor_timer = 900;//29900;	 /**< Sensor Timer **/
	cfg.wdt_timer = 1000;	 /**< WDT Timer **/
#endif
	cfg.nframeend_ctrl = PMU_NFRAMEEND_CTRL_I2C;
	cfg.cm55s_reset = reset;
	cfg.pmu_pad_pa01_mask = pmu_pad_pa01_mask;
	cfg.pmu_rtc_mask = pmu_rtc_mask;
	cfg.pmu_pad_pa23_mask = pmu_pad_pa23_mask;			/**< PMU SB GPIO Interrupt Mask **/
	cfg.pmu_i2cw_mask = pmu_i2cw_mask;			/**< PMU I2C Wakeup Interrupt Mask **/
	cfg.pmu_timer_mask = pmu_timer_mask;			/**< PMU Timer0~5 Wakeup Interrupt Mask  **/
	cfg.pmu_cmp_mask = pmu_cmp_mask;			/**< PMU CMP Wakeup Interrupt Mask  **/
	cfg.pmu_ts_mask = pmu_ts_mask;			/**< PMU TS Wakeup Interrupt Mask  **/
	cfg.pmu_anti_mask = pmu_anti_mask;
	cfg.pmu_mipii2c_noack_mask = 0;
	cfg.pmu_senint_mask = pmu_senint_mask;		/**< PMU SENSOR Interrupt Mask **/
	cfg.support_debugdump = support_debugdump;

	cfg.tcm_retention = retention;			/**< CM55M TCM Retention**/
	cfg.hscsram_retention[0] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[1] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[2] = retention;	/**< HSC SRAM Retention**/
	cfg.hscsram_retention[3] = retention;	/**< HSC SRAM Retention**/
	cfg.lscsram_retention = 1;		//cfg.lscsram_retention = retention;		/**< LSC SRAM Retention**/
	cfg.skip_bootflow.sec_mem_flag = retention;			/**< Skip Boot Flow**/
	cfg.skip_bootflow.first_bl_flag = retention; /*!< First BL Retention */
	cfg.skip_bootflow.cm55m_s_app_flag = retention; /*!< cm55m_s_app Retention */
	cfg.skip_bootflow.cm55m_ns_app_flag = retention; /*!< cm55m_ns_app Retention */
	cfg.skip_bootflow.cm55s_s_app_flag = 1;//cfg.skip_bootflow.cm55s_s_app_flag = retention; /*!< cm55s_s_app Retention */
	cfg.skip_bootflow.cm55s_ns_app_flag = 1;//cfg.skip_bootflow.cm55s_ns_app_flag = retention; /*!< cm55s_ns_app Retention */
	cfg.skip_bootflow.cm55m_model_flag = retention; /*!< cm55m model Retention */
	cfg.skip_bootflow.cm55s_model_flag = 1;//cfg.skip_bootflow.cm55s_model_flag = retention; /*!< cm55s model Retention */
	cfg.skip_bootflow.cm55m_appcfg_flag = retention; /*!< cm55m appcfg Retention */
	cfg.skip_bootflow.cm55s_appcfg_flag = 1;//cfg.skip_bootflow.cm55s_appcfg_flag = retention; /*!< cm55s appcfg Retention */
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = retention;/*!< cm55m_s_app RW Data Retention */
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = retention;/*!< cm55m_ns_app RW Data Retention */
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = 1;//cfg.skip_bootflow.cm55s_s_app_rwdata_flag = retention;/*!< cm55s_s_app RW Data Retention */
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = 1;//cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = retention;/*!< cm55s_ns_app RW Data Retention */

	cfg.fast_vpr = 0;
	cfg.pmu_dcdc_outpin = dcdcpin;

	hx_lib_pm_cfg_set(&cfg, NULL, PM_MODE_PS_NOVID_PREROLLING);

	SetAlarmPMU();

	hsc_cfg.hscclk.hscclksrc = SCU_HSCCLKSRC_RC24M1M;
	hsc_cfg.hscclk.hscclkdiv = SCU_HSCCLKDIV_1;
	hsc_cfg.hscd12clksrc = SCU_HSCD12CLKSRC_HSC;
	hsc_cfg.i3chcdiv = SCU_HSCI3CHCLKDIV_1;
	hsc_cfg.sdiodiv = SCU_HSCSDIOCLKDIV_1;
	lsc_cfg.lscclksrc = SCU_LSCCLKSRC_RC24M1M;
	lsc_cfg.lscclkdiv = SCU_LSCCLKDIV_1;

	if(trigger == 1)
	{
		hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYAPP);
	}

}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int tflm_nycu_z_axsis_app(void) {

	uint32_t wakeup_event;
	uint32_t wakeup_event1;
	uint32_t freq=0;

	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
	hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);

    hx_drv_swreg_aon_get_pllfreq(&freq);
    xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x, freq=%d\n", wakeup_event, wakeup_event1, freq);

    pinmux_init();

    //SCB_DisableICache();
    //SCB_DisableDCache();
    //cold boot
	if((wakeup_event == PMU_WAKEUP_NONE) && (wakeup_event1 == PMU_WAKEUPEVENT1_NONE)) {
	}
	else {
		hx_lib_pm_ctrl_fromPMUtoCPU(NULL);
	}

	hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    //hx_lib_spi_eeprom_open_speed(USE_DW_SPI_MST_Q, 6000000);

	hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);

	//
// #ifdef WATCHDOG_VERSION

// 	//watch dog start
// 	WATCHDOG_CFG_T wdg_cfg;
// 	wdg_cfg.period = WATCH_DOG_TIMEOUT_TH;
// 	wdg_cfg.ctrl = WATCHDOG_CTRL_CPU;
// 	wdg_cfg.state = WATCHDOG_STATE_DC;
// 	wdg_cfg.type = WATCHDOG_RESET;//wewweWATCHDOG_INT;
// 	hx_drv_watchdog_start(WATCHDOG_ID_0, &wdg_cfg , WDG_Reset_ISR_CB);
// 	xprintf("hx_drv_watchdog_start\n");
// #endif

	//check current use case
	// //check current use case
	//judge which case

	hx_drv_swreg_aon_get_appused1(&judge_case_data);
	//transfer type
	g_trans_type = (judge_case_data>>16);
	//model case 
	g_use_case = (judge_case_data&0xff);


#ifndef CPU_24MHZ_VERSION
	xprintf("ori_clk src info, 0x56100030=%x\n",EPII_get_memory(0x56100030));
	xprintf("ori_clk src info, 0x56100034=%x\n",EPII_get_memory(0x56100034));
	xprintf("ori_clk src info, 0x56100038=%x\n",EPII_get_memory(0x56100038));

	EPII_set_memory(0x56100030,0x4037);
	EPII_set_memory(0x56100034,0x0);
	EPII_set_memory(0x56100038,0xc1b8);

	xprintf("clk src info, 0x56100030=%x\n",EPII_get_memory(0x56100030));
	xprintf("clk src info, 0x56100034=%x\n",EPII_get_memory(0x56100034));
	xprintf("clk src info, 0x56100038=%x\n",EPII_get_memory(0x56100038));
#endif
	// mm_set_initial(BOOT2NDLOADER_BASE, 0x00200000-(BOOT2NDLOADER_BASE-0x34000000));
#ifdef __GNU__
	xprintf("__GNUC \n");
	extern char __mm_start_addr__;
	xprintf("__mm_start_addr__ address: %x\r\n",&__mm_start_addr__);
	mm_set_initial((int)(&__mm_start_addr__), 0x00200000-((int)(&__mm_start_addr__)-0x34000000));
#else
	static uint8_t mm_start_addr __attribute__((section(".bss.mm_start_addr")));
	xprintf("mm_start_addr address: %x \r\n",&mm_start_addr);
	mm_set_initial((int)(&mm_start_addr), 0x00200000-((int)(&mm_start_addr)-0x34000000));
#endif
	if(g_use_case == 0) {
		xprintf("yolov8n pose  \n");
#ifdef EN_ALGO
		cv_nycu_z_axsis_init(true, true, NYCU_Z_AXSIS_FLASH_ADDR);
#endif
		while(1)
		{
			cv_nycu_z_axsis_run();
		}


	}
	return 0;
}
