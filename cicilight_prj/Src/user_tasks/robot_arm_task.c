
//机械手臂舵机2任务
static void servo2_task(void const * argument)
{
 osEvent msg;
 APP_LOG_INFO("++++++机械手臂舵机2任务开始！\r\n");
 while(1)
 {
 msg= osMessageGet(servo2_msg_queue_hdl,osWaitForever);
 if(msg.status!=osEventMessage)
 {
  APP_LOG_INFO("手臂舵机2收到错误消息！\r\n");
  continue ;
 }
 APP_LOG_DEBUG("手臂舵机2收到消息角度：%d°！\r\n",msg.value.v);
 HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
 MX_TIM2_ReInit_CH3((msg.value.v*11+500)/10);
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
 osDelay(SERVO2_ENABLE_TIME_VALUE);
 osSignalSet(sync_task_hdl,SERVO2_REACH_POS_OK_SIGNAL);
 }
}