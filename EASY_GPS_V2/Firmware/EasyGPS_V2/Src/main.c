/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "nmea.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t u1rxchar[1];
uint8_t u2txbuf[100];
uint8_t gpsbuf[100];
uint8_t gpsstr[100];
uint8_t gps_read_ok = 0;
uint8_t gps_info_update = 0;
uint8_t gpslen = 0;
GPS_INFO_t gps_info;
uint8_t gps_info_send_buffer[59];
uint8_t gps_info_send_buffer_chksum = 0;
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    SystemClock_Config();

    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, u1rxchar, 1);
    gps_info_send_buffer[0] = 0xAA;
    gps_info_send_buffer[1] = 0x55;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if(gps_read_ok)
        {
            memcpy(gpsstr, gpsbuf, gpslen);
            gps_read_ok = 0;
            //HAL_UART_Transmit_IT(&huart2, gpsstr, gpslen);
            HAL_GPIO_TogglePin(GPIOB, LD1_Pin);

            /*updata gps information*/
            if(strncmp((const char*)gpsstr, "$GNRMC,", 7) == 0)
            {
                nmea_parse_gprmc((char*)gpsstr, &gps_info);
                gps_info_update++;
            }

            if(strncmp((const char*)gpsstr, "$GNGGA,", 7) == 0)
            {
                nmea_parse_gpgga((char*)gpsstr, &gps_info);
                gps_info_update++;
            }

            if(strncmp((const char*)gpsstr, "$GNVTG,", 7) == 0)
            {
                nmea_parse_gpvtg((char*)gpsstr, &gps_info);
                gps_info_update++;
            }

//            if(gps_info.quality != '0')
//            {
//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_SET);
//            }
//            else
//            {
//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  GPIO_PIN_RESET);
//            }

            /* Debug message :
            sprintf(u2txbuf,"\x1b[2J");
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"\x1b[0;0H");
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);

            sprintf(u2txbuf,"\r\nNMEA Parser Result--------------------------\r\n");
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Time: %02d:%02d:%02d\r\n",gps_info.hour,gps_info.minuate,gps_info.sec);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Position:<Lat>%f   <Lng>%f\r\n",gps_info.latitude,gps_info.longitude);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Altitude:%.2f\r\n",gps_info.altitude);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Number of Satellites:%d  Fix Type:%d  HDOP:%.2f\r\n",gps_info.satellites,gps_info.quality,gps_info.hdop);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Heading: %.2f\r\n",gps_info.course);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            sprintf(u2txbuf,"Ground Speed:%.2f km/h\r\n",gps_info.speed);
            HAL_UART_Transmit(&huart2,(uint8_t*)u2txbuf ,strlen(u2txbuf),10000);
            */
            /*send package to uart*/
            if(gps_info_update > 1)
            {
                gps_info_update = 0;
                memcpy(gps_info_send_buffer + 2, &gps_info, sizeof(gps_info));
                gps_info_send_buffer_chksum = 0;

                for(int i = 0; i < 56; i++)
                {
                    gps_info_send_buffer_chksum += gps_info_send_buffer[i + 2];
                }

                gps_info_send_buffer[58] = gps_info_send_buffer_chksum;
                HAL_UART_Transmit(&huart2, (uint8_t*)gps_info_send_buffer , 59, 10000);
            }

        }

    }

    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RSTG_GPIO_Port, RSTG_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : RSTG_Pin */
    GPIO_InitStruct.Pin = RSTG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RSTG_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD1_Pin LD2_Pin */
    GPIO_InitStruct.Pin = LD1_Pin | LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static int gps_receive_status = 0;
    static int gps_receive_index = 0;

    if(huart->Instance == huart1.Instance)
    {
        if(gps_read_ok)
        {
            HAL_UART_Receive_IT(huart, u1rxchar, 1);
            return;
        }

        switch(gps_receive_status)
        {
        case 0:
            if(u1rxchar[0] == '$')
            {
                gpsbuf[gps_receive_index] = u1rxchar[0];
                gps_receive_index++;
                gps_receive_status++;
            }

            break;

        case 1:
            if(u1rxchar[0] != '\n')
            {
                gpsbuf[gps_receive_index] = u1rxchar[0];
                gps_receive_index++;
            }
            else
            {
                gpsbuf[gps_receive_index++] = u1rxchar[0];
                gpslen = gps_receive_index;
                gps_receive_index = 0;
                gps_receive_status = 0;
                gps_read_ok = 1;
            }

            break;

        }

        HAL_UART_Receive_IT(&huart1, u1rxchar, 1);

    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
//    while(1)
//    {

//    }

    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
