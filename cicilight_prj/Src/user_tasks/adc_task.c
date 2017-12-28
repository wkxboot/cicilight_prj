#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "JJDK_ZK_GZ1.h"
#include "juice_common.h"
#include "mb_reg.h"
#include "adc_task.h"
#define APP_LOG_MODULE_NAME   "[adc_task]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_DEBUG  
#include "app_log.h"
#include "app_error.h"

static void adc_process_sample_average()
{
 adc_result[0]=adc_average[ADC_PRESSER_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//压杯电机
 adc_result[1]=adc_average[ADC_OH_DOOR_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//升降门
 adc_result[2]=adc_average[ADC_24V_IDX]*330000/(4096*OC_SCALE*RES_VALUE);//24V
 adc_result[3]=BSP_get_temperature(adc_average[ADC_T_IDX]);//温度
 adc_result[4]=adc_average[ADC_BEMF_IDX]*3300*ADC_BEMF_DIV/4096;//BEMF
#if  1
 APP_LOG_INFO("压杯电流：%d mA.\r\n",adc_result[0]);
 APP_LOG_INFO("升降门电流：%d mA.\r\n",adc_result[1]);
 APP_LOG_INFO("24V电流：%d mA.\r\n",adc_result[2]);
 APP_LOG_INFO("温度值：%d ℃.\r\n",(int16_t)adc_result[3]);
 APP_LOG_INFO("BEMF：%d mV.\r\n",adc_result[4]);
#endif
}
 
static uint8_t juice_is_presser_oc()
{
 if(adc_result[0] > ADC_PRESSER_OC_THRESHOLD_mAMPERE)
 { 
 APP_LOG_ERROR("压杯电机过载：%d mA！\r\n",adc_result[0]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}
static uint8_t juice_is_oh_door_oc()
{
 if(adc_result[1] > ADC_OH_DOOR_OC_THRESHOLD_mAMPERE)
 {
 APP_LOG_ERROR("升降门过载:%d mA！\r\n",adc_result[1]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}
static uint8_t juice_is_24v_oc()
{
 if(adc_result[2] > ADC_24V_OC_THRESHOLD_mAMPERE)
 {
 APP_LOG_ERROR("24V过载:%d mA！\r\n",adc_result[2]);
 return JUICE_TRUE;
 }

 return JUICE_FALSE;
}

static uint8_t juice_is_column_step_motor_stall()//bemf 反向电动势
{
 if(adc_result[4] < ADC_BEMF_THRESHOLD_mVOLTAGE)
 {
 APP_LOG_ERROR("60步进电机堵转：%d mV！\r\n",adc_result[4]);
 return JUICE_TRUE;
 }
 return JUICE_FALSE;
}

static void adc_task(void const * argument)
{
 uint32_t adc_cusum[ADC_CNT]={0};
 uint32_t adc_timeout=0;
 uint16_t adc_times=0;
 APP_LOG_INFO("++++++ADC任务开始！\r\n");

 while(1)
 {
 while(adc_timeout<ADC_TIMEOUT_VALUE)
 {
 //adc
 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_sample,ADC_CNT);
 osDelay(ADC_TASK_RUN_INTERVAL_VALUE);
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_cusum[i]+=adc_sample[i];  
 }
 adc_timeout+=ADC_TASK_RUN_INTERVAL_VALUE;
 adc_times++;
 }
 APP_LOG_INFO("ADC取样时间：%d ms，取样次数：%d.\r\n",adc_timeout,adc_times);
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_average[i]=adc_cusum[i]/adc_times; 
 //APP_LOG_DEBUG("累加值[%d]：%d.\r\n",i,adc_cusum[i]);
 //APP_LOG_INFO("平均值[%d]：%d.\r\n",i,adc_average[i]);
 } 
 adc_process_sample_average();
 
 adc_timeout=0;
 adc_times=0; 
 for(uint8_t i=0;i<ADC_CNT;i++)
 {
 adc_cusum[i]=0;  
 }
 }
}

