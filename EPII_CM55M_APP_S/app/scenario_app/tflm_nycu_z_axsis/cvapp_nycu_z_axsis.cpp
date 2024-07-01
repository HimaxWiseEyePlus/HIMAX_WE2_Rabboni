/*
 * cvapp.cpp
 *
 *  Created on: 2018�~12��4��
 *      Author: 902452
 */

#include <cstdio>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "WE2_device.h"
#include "board.h"
#include "cvapp_nycu_z_axsis.h"
#include "cisdp_sensor.h"

#include "WE2_core.h"

#include "ethosu_driver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#include "img_proc_helium.h"


#include "xprintf.h"
#include "spi_master_protocol.h"
#include "cisdp_cfg.h"
#include "memory_manage.h"

#include "send_result.h"

#define INPUT_TENSOR_WIDTH   128

#define INPUT_TENSOR_CHANNELS 3



#define  EACH_STEP_TICK 0
#define TOTAL_STEP_TICK 1

#ifdef TRUSTZONE_SEC
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE	BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE	BASE_ADDR_APB_U55_CTRL
#endif
#endif


using namespace std;

namespace {


constexpr int tensor_arena_size = 3*1024;



static uint32_t tensor_arena=0;

struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */
tflite::MicroInterpreter *nycu_z_axsis_int_ptr=nullptr;
TfLiteTensor *nycu_z_axsis_input, *nycu_z_axsis_output;

};


#define CPU_CLK	0xffffff+1
uint32_t systick_1, systick_2;
uint32_t loop_cnt_1, loop_cnt_2;
static uint32_t capture_image_tick = 0;
int continuous_inference = 0;
int last_estimate_label = 3;
int consecutiveInferenceThresholds[3] = {2, 2, 1};
float ValidInferenceProbabilityThresholds[3] = {0.8, 0.8, 0.4};
// #endif

static void _arm_npu_irq_handler(void)
{
    /* Call the default interrupt handler from the NPU driver */
    ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void _arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;

    /* Register the EthosU IRQ handler in our vector table.
     * Note, this handler comes from the EthosU driver */
    EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);

    /* Enable the IRQ */
    NVIC_EnableIRQ(ethosu_irqnum);

}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
    int err = 0;

    /* Initialise the IRQ */
    _arm_npu_irq_init();

    /* Initialise Ethos-U55 device */
    const void * ethosu_base_address = (void *)(U55_BASE);

    if (0 != (err = ethosu_init(
                            &ethosu_drv,             /* Ethos-U driver device pointer */
                            ethosu_base_address,     /* Ethos-U NPU's base address. */
                            NULL,       /* Pointer to fast mem area - NULL for U55. */
                            0, /* Fast mem region size. */
							security_enable,                       /* Security enable. */
							privilege_enable))) {                   /* Privilege enable. */
    	xprintf("failed to initalise Ethos-U device\n");
            return err;
        }

    xprintf("Ethos-U55 device initialised\n");

    return 0;
}


int cv_nycu_z_axsis_init(bool security_enable, bool privilege_enable, uint32_t model_addr) {
	int ercode = 0;

	//set memory allocation to tensor_arena
	tensor_arena = mm_reserve_align(tensor_arena_size,0x20); //1mb
	xprintf("TA[%x]\r\n",tensor_arena);


	if(_arm_npu_init(security_enable, privilege_enable)!=0)
		return -1;

	if(model_addr != 0) {
		static const tflite::Model*nycu_z_axsis_model = tflite::GetModel((const void *)model_addr);

		if (nycu_z_axsis_model->version() != TFLITE_SCHEMA_VERSION) {
			xprintf(
				"[ERROR] nycu_z_axsis_model's schema version %d is not equal "
				"to supported version %d\n",
				nycu_z_axsis_model->version(), TFLITE_SCHEMA_VERSION);
			return -1;
		}
		else {
			xprintf("nycu_z_axsis_model model's schema version %d\n", nycu_z_axsis_model->version());
		}

		static tflite::MicroErrorReporter nycu_z_axsis_micro_error_reporter;
		static tflite::MicroMutableOpResolver<1> nycu_z_axsis_op_resolver;


		if (kTfLiteOk != nycu_z_axsis_op_resolver.AddEthosU()){
			xprintf("Failed to add Arm NPU support to op resolver.");
			return false;
		}

		static tflite::MicroInterpreter nycu_z_axsis_static_interpreter(nycu_z_axsis_model, nycu_z_axsis_op_resolver,
				(uint8_t*)tensor_arena, tensor_arena_size, &nycu_z_axsis_micro_error_reporter);

		if(nycu_z_axsis_static_interpreter.AllocateTensors()!= kTfLiteOk) {
			return false;
		}
		nycu_z_axsis_int_ptr = &nycu_z_axsis_static_interpreter;
		nycu_z_axsis_input = nycu_z_axsis_static_interpreter.input(0);
        
		nycu_z_axsis_output = nycu_z_axsis_static_interpreter.output(0);
        

	}
	xprintf("initial done\n");
	return ercode;
}

void print_float(float f_z)
{
	float o_f_z = f_z;
	short D_z, D_f_z;
	D_z = (short)f_z;
	f_z -=  D_z;

	if(f_z <=0)
		f_z = 0-f_z;

	D_f_z = f_z*1000;
	if(D_z==0 && o_f_z < 0)printf("-%d.%03d\r\n",D_z, D_f_z);
	else printf("%d.%03d\r\n",D_z, D_f_z);
}

void check_estimate_label(int output_label, float* output_deq_value)
{


	if (output_label != last_estimate_label)
	{
        continuous_inference = 0;
	}
	if (output_deq_value[output_label] > ValidInferenceProbabilityThresholds[output_label])
	{
		continuous_inference += 1;
	}
	if (continuous_inference > consecutiveInferenceThresholds[output_label])
	{
		if (output_label == 0)
		{
			printf("@@ WE2 up\r\n");
		}

		else if( output_label == 1)
		{
			printf("@@ WE2 down\r\n");
		}

		else if( output_label == 2)
		{
			printf("@@ WE2 no move\r\n");
		}

		else
		{
			
		}
	}
		
	
	last_estimate_label = output_label;

 
}
int cv_nycu_z_axsis_run() {
	int ercode = 0;


	#if TOTAL_STEP_TICK
		SystemGetTick(&systick_1, &loop_cnt_1);
	#endif

    if(nycu_z_axsis_int_ptr!= nullptr) {
    	
        DEV_UART* console_uart;
		console_uart = hx_drv_uart_get_dev((USE_DW_UART_E)CONSOLE_UART_ID);
		console_uart->uart_open(UART_BAUDRATE_921600);

		// float rbuffer[128*3];
		// memset(rbuffer, 0 , sizeof(float)*128*3);
		// console_uart->uart_read(rbuffer , sizeof(float)*128*3);
		// hx_CleanDCache_by_Addr((volatile void *) rbuffer,  sizeof(float)*128*3);

		/////////////////////////////////////////////////////////////////////////////
		float r_accX_data_buffer[128];
		memset(r_accX_data_buffer, 0 , sizeof(float)*128);
		console_uart->uart_read(r_accX_data_buffer , sizeof(float)*128);
		hx_CleanDCache_by_Addr((volatile void *) r_accX_data_buffer,  sizeof(float)*128);

		float r_accY_data_buffer[128];
		memset(r_accY_data_buffer, 0 , sizeof(float)*128);
		console_uart->uart_read(r_accY_data_buffer , sizeof(float)*128);
		hx_CleanDCache_by_Addr((volatile void *) r_accY_data_buffer,  sizeof(float)*128);

		float r_accZ_data_buffer[128];
		memset(r_accZ_data_buffer, 0 , sizeof(float)*128);
		console_uart->uart_read(r_accZ_data_buffer , sizeof(float)*128);
		hx_CleanDCache_by_Addr((volatile void *) r_accZ_data_buffer,  sizeof(float)*128);

		float temp_pre_rbuffer[128*3];
		for (int i = 0; i < 128; ++i) {
			
			temp_pre_rbuffer[i * 3] = r_accX_data_buffer[i] / 8.0;

			temp_pre_rbuffer[i * 3 + 1] = r_accY_data_buffer[i] / 8.0;

			temp_pre_rbuffer[i * 3 + 2] = r_accZ_data_buffer[i] / 8.0;
		}
		#if 0
			for(int i = 0 ;i < 10; i++)
			{
				printf("WE2 r_accX_data_buffer[%d]: ",i);
				print_float(r_accX_data_buffer[i]);
				printf("WE2 r_accY_data_buffer[%d]: ",i);
				print_float(r_accY_data_buffer[i]);
				printf("WE2 r_accZ_data_buffer[%d]: ",i);
				print_float(r_accZ_data_buffer[i]);
			}
		#endif
		////////////////////////////////////////////////////////////////////////////////////

		#if 0
			// for(int i = 0 ;i < 5; i++)
			// {
			// 	printf("WE2 rbuffer[%d]: ",i);
			// 	print_float(rbuffer[i]);
			// }

			for(int i = 0 ;i < 5; i++)
			{
				// printf("WE2 rbuffer[%d]: ",i);
				// print_float(rbuffer[i]);
				printf("WE2 temp_pre_rbuffer[%d]: ",i);
				print_float(temp_pre_rbuffer[i]);
			}
		#endif
		#if EACH_STEP_TICK
		SystemGetTick(&systick_1, &loop_cnt_1);
        #endif

		float intput_zero_point = ((TfLiteAffineQuantization*)(nycu_z_axsis_input->quantization.params))->zero_point->data[0];
		float intput_scale = ((TfLiteAffineQuantization*)(nycu_z_axsis_input->quantization.params))->scale->data[0];
		// printf("WE2 intput_scale:");
		// print_float(intput_scale);
		// printf("WE2 intput_zero_point:");
		// print_float(intput_zero_point);
		for (int i = 0; i < nycu_z_axsis_input->bytes; ++i) {
			*((int8_t *)nycu_z_axsis_input->data.data+i) = (int8_t)((temp_pre_rbuffer[i]/ intput_scale) + intput_zero_point);
			// if(i<6)printf("WE2 nycu_z_axsis_input->data.data[%d]= %d\r\n",i,*((int8_t *)nycu_z_axsis_input->data.data+i));
    	}


        #if EACH_STEP_TICK
            SystemGetTick(&systick_2, &loop_cnt_2);
            xprintf("Tick for Invoke for convert floating input data to int8 for nycu_z_axsis:[%d]\r\n\n",(loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2));    
        #endif	


        #if EACH_STEP_TICK
		SystemGetTick(&systick_1, &loop_cnt_1);
        #endif

		TfLiteStatus invoke_status = nycu_z_axsis_int_ptr->Invoke();

        #if EACH_STEP_TICK
			SystemGetTick(&systick_2, &loop_cnt_2);
			xprintf("Tick for invoke of nycu_z_axsis:[%d]\r\n",(loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2));	
        #endif

		if(invoke_status != kTfLiteOk)
		{
			xprintf("nycu_z_axsis invoke fail\n");
			return -1;
		}
		else
		{
			#if DBG_APP_LOG
			xprintf("nycu_z_axsis invoke pass\n");
			#endif
		}

        #if EACH_STEP_TICK
            SystemGetTick(&systick_1, &loop_cnt_1);
        #endif
		//retrieve output data
		int output_label = 0;
		int max_output_value = -128;
		float output_deq_value[3];
		for (int i = 0; i < nycu_z_axsis_output->bytes; ++i) {
			/***if only want to compare the value about output we do not need to dequantize****/
			int value =  nycu_z_axsis_output->data.int8[ i];
			if(value > max_output_value)
			{
				max_output_value = value;
				output_label = i;
			}
			/***if only want to compare the value about output we do not need to dequantize****/
			float output_zero_point = ((TfLiteAffineQuantization*)(nycu_z_axsis_output->quantization.params))->zero_point->data[0];
			float output_scale = ((TfLiteAffineQuantization*)(nycu_z_axsis_output->quantization.params))->scale->data[0];
			float deq_value = ((float)value - (float)output_zero_point) * (float)output_scale;
			output_deq_value[i] = deq_value;
			
		}
		printf("WE2 output[0]: %d WE2 output[1]: %d WE2 output[2]: %d \r\n",nycu_z_axsis_output->data.int8[0],nycu_z_axsis_output->data.int8[1],nycu_z_axsis_output->data.int8[2]);
		for(int i=0;i<3;i++)
		{
			printf("WE2 deq_value output[%d]:",i);
			print_float(output_deq_value[i]);
		}
		printf("@@@@@ WE2 estimate label:  %d @@@@\r\n",output_label);

		check_estimate_label(output_label,output_deq_value);
		#if EACH_STEP_TICK
			SystemGetTick(&systick_2, &loop_cnt_2);
			xprintf("Tick for Invoke for nycu_z_axsis post_processing:[%d]\r\n\n",(loop_cnt_2-loop_cnt_1)*CPU_CLK+(systick_1-systick_2));    
        #endif

		#if DBG_APP_LOG
			xprintf("nycu_z_axsis done\r\n");
		#endif
    }


#ifdef UART_SEND_ALOGO_RESEULT
	#if TOTAL_STEP_TICK						
		SystemGetTick(&systick_2, &loop_cnt_2);
		
	#endif

	
#endif	
	


	return ercode;
}

int cv_nycu_z_axsis_deinit()
{
	
	return 0;
}

