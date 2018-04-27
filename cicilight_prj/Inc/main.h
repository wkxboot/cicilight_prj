/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BSP_RUN_LED_POS_Pin GPIO_PIN_2
#define BSP_RUN_LED_POS_GPIO_Port GPIOE
#define BSP_COLUMN_STEP_MOTOR_RST_POS_Pin GPIO_PIN_6
#define BSP_COLUMN_STEP_MOTOR_RST_POS_GPIO_Port GPIOE
#define BSP_ROW_STEP_MOTOR_RST_POS_Pin GPIO_PIN_13
#define BSP_ROW_STEP_MOTOR_RST_POS_GPIO_Port GPIOC
#define BSP_CUP_PRESS_OK_POS_Pin GPIO_PIN_14
#define BSP_CUP_PRESS_OK_POS_GPIO_Port GPIOC
#define BSP_CUP_PRESSER_IN_BOT_POS_Pin GPIO_PIN_15
#define BSP_CUP_PRESSER_IN_BOT_POS_GPIO_Port GPIOC
#define BSP_CUP_PRESSER_IN_TOP_POS_Pin GPIO_PIN_0
#define BSP_CUP_PRESSER_IN_TOP_POS_GPIO_Port GPIOF
#define BSP_OH_DOOR_CLAMP_HAND_POS_Pin GPIO_PIN_1
#define BSP_OH_DOOR_CLAMP_HAND_POS_GPIO_Port GPIOF
#define BSP_MS_7_POS_Pin GPIO_PIN_2
#define BSP_MS_7_POS_GPIO_Port GPIOF
#define BSP_MS_8_POS_Pin GPIO_PIN_3
#define BSP_MS_8_POS_GPIO_Port GPIOF
#define BSP_MS_9_POS_Pin GPIO_PIN_4
#define BSP_MS_9_POS_GPIO_Port GPIOF
#define BSP_MS_10_POS_Pin GPIO_PIN_5
#define BSP_MS_10_POS_GPIO_Port GPIOF
#define BSP_OH_DOOR_HAND_DETECT_POS_Pin GPIO_PIN_13
#define BSP_OH_DOOR_HAND_DETECT_POS_GPIO_Port GPIOF
#define BSP_OH_DOOR_IN_TOP_POS_Pin GPIO_PIN_14
#define BSP_OH_DOOR_IN_TOP_POS_GPIO_Port GPIOF
#define BSP_OH_DOOR_IN_BOT_POS_Pin GPIO_PIN_15
#define BSP_OH_DOOR_IN_BOT_POS_GPIO_Port GPIOF
#define BSP_ROW_POS_SENSOR_Pin GPIO_PIN_0
#define BSP_ROW_POS_SENSOR_GPIO_Port GPIOG
#define BSP_COLUMN_POS_SENSOR_Pin GPIO_PIN_1
#define BSP_COLUMN_POS_SENSOR_GPIO_Port GPIOG
#define BSP_CUP_POS_Pin GPIO_PIN_7
#define BSP_CUP_POS_GPIO_Port GPIOE
#define BSP_COLUMN_STEP_MOTOR_CS_POS_Pin GPIO_PIN_8
#define BSP_COLUMN_STEP_MOTOR_CS_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_DIR_POS_Pin GPIO_PIN_9
#define BSP_COLUMN_STEP_MOTOR_DIR_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_RESET_POS_Pin GPIO_PIN_10
#define BSP_COLUMN_STEP_MOTOR_RESET_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_SLEEP_POS_Pin GPIO_PIN_11
#define BSP_COLUMN_STEP_MOTOR_SLEEP_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_FAULT_POS_Pin GPIO_PIN_12
#define BSP_COLUMN_STEP_MOTOR_FAULT_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_STALL_POS_Pin GPIO_PIN_13
#define BSP_COLUMN_STEP_MOTOR_STALL_POS_GPIO_Port GPIOD
#define BSP_COLUMN_STEP_MOTOR_STEP_POS_Pin GPIO_PIN_6
#define BSP_COLUMN_STEP_MOTOR_STEP_POS_GPIO_Port GPIOC
#define BSP_ROW_STEP_MOTOR_STEP_POS_Pin GPIO_PIN_7
#define BSP_ROW_STEP_MOTOR_STEP_POS_GPIO_Port GPIOC
#define BSP_ROW_STEP_MOTOR_DIR_POS_Pin GPIO_PIN_8
#define BSP_ROW_STEP_MOTOR_DIR_POS_GPIO_Port GPIOC
#define BSP_ROW_STEP_MOTOR_EN_POS_Pin GPIO_PIN_9
#define BSP_ROW_STEP_MOTOR_EN_POS_GPIO_Port GPIOC
#define BSP_ROW_STEP_MOTOR_BRAKE_POS_Pin GPIO_PIN_8
#define BSP_ROW_STEP_MOTOR_BRAKE_POS_GPIO_Port GPIOA
#define BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_Pin GPIO_PIN_5
#define BSP_PRESS_MOTOR_POSITIVE_RELAY_POS_GPIO_Port GPIOB
#define BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_Pin GPIO_PIN_6
#define BSP_PRESS_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port GPIOB
#define BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_Pin GPIO_PIN_7
#define BSP_OH_DOOR_MOTOR_POSITIVE_RELAY_POS_GPIO_Port GPIOB
#define BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_Pin GPIO_PIN_8
#define BSP_OH_DOOR_MOTOR_NEGATIVE_RELAY_POS_GPIO_Port GPIOB
#define BSP_ENVIRONMENT_RELAY_POS_Pin GPIO_PIN_9
#define BSP_ENVIRONMENT_RELAY_POS_GPIO_Port GPIOB
#define BSP_JUICING_MOTOR_RELAY_POS_Pin GPIO_PIN_0
#define BSP_JUICING_MOTOR_RELAY_POS_GPIO_Port GPIOE
#define BSP_COMPRESSOR_RELAY_POS_Pin GPIO_PIN_1
#define BSP_COMPRESSOR_RELAY_POS_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
