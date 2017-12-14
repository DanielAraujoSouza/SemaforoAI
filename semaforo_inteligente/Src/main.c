/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
int btn_s1 = 0;
int btn_s2 = 0;
int uss1_ciclos_inativo = 0;
int uss2_ciclos_inativo = 0;
int tm_frequencia = 10; // leituras por segundo
int uss1_calibragem = 1; //distancia padrao
int uss2_calibragem = 1; //distancia padrao
int uss1_fluxo = 0; //carros que cruzaram o semaforo 1
int uss2_fluxo = 0; //carros que cruzaram o semaforo 2
int uss1_resetado = 1;
int uss2_resetado = 1;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); //S1R
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //S1Y
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //P1G

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET); //S2R
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET); //S2Y
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); //S2G
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //P2G
  while (1)
  {
  /* USER CODE END WHILE */
  //semaforo 1
	  for (int i = 0; i < 75000; i++){
		  // botao de pedestre precionado
		  if(btn_s1 > 0){
			  //Semaforo 1 sem trafego
			  if(uss1_ciclos_inativo/tm_frequencia > 5){
				  break;
			  }
			  //Fecha semaforo no tempo normal
			  else if(i >= 25000){
				  break;
			  }
		  }
		  //Verificação de inatividade
		  else if (uss1_ciclos_inativo/tm_frequencia > 5 && uss2_ciclos_inativo/tm_frequencia <= 5){
			  break;
		  }
		  //Verifica situação para overtime
		  else if(i >= 25000){
			  if(uss2_ciclos_inativo/tm_frequencia <= 5){
				  break;
			  }
		  }
		  Delay_Us(1000); //1ms
	  }
	  troca_semaforo(1);
	  //semaforo 2
	  for (int i = 0; i < 75000; i++){
		  // botao de pedestre precionado
		  if(btn_s2 >= 0){
			  //Semaforo 2 sem trafego
			  if(uss2_ciclos_inativo/tm_frequencia > 5){
				  break;
			  }
			  //Fecha semaforo no tempo normal
			  else if(i >= 25000){
				  break;
			  }
		  }
		  //Verificação de inatividade
		  else if (uss2_ciclos_inativo/tm_frequencia > 5 && uss1_ciclos_inativo/tm_frequencia <= 5){
			  break;
		  }
		  //Verifica situação para overtime
		  else if(i >= 25000){
			  if(uss2_ciclos_inativo/tm_frequencia <= 5){
				  break;
			  }
		  }
		  Delay_Us(1000);
	  }
	  troca_semaforo(2);
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
uint32_t getUs(void) {
	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());

	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void Delay_Us(uint32_t micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// faz a leitura dos botoes e dos sensores
    if (htim->Instance==TIM2)
	{
    	//envia um pulso para trigger dos sensores durante 10us
    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //USS1 trigger
    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET); //USS2 trigger
    	Delay_Us(10);
    	//conta os ciclos de echo para calcular a distancia
    	int uss1_delay = 0;
    	int uss2_delay = 0;
    	for(int i = 0; i < 60000; i++){
    		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_SET){
    			uss1_delay++;
    		}
    		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3) == GPIO_PIN_SET){
    			uss2_delay++;
			}
    		Delay_Us(1);
    	}
    	float distancia1 = (float)(uss1_delay*343)/1000;
    	//float distancia1 = (uss1_delay*10)/55.246; //Distancia em centimetros
    	if(distancia1 > uss1_calibragem){
    		if(uss1_resetado == 1){
    			uss1_fluxo++;
    		}
    		uss1_resetado = 0;
    		uss2_ciclos_inativo = 0;
    	}
    	else{
    		uss1_resetado = 1;
    		uss1_ciclos_inativo++;
    	}
    	float distancia1 = (float)(uss2_delay*343)/1000;
    	//float distancia2 = (uss2_delay*10)/55.246;//Distancia em centimetros
		if(distancia2 > uss2_calibragem){
			if(uss2_resetado == 1){
				uss2_fluxo++;
			}
			uss2_resetado = 0;
			uss2_ciclos_inativo = 0;
		}
		else{
			uss2_resetado = 1;
			uss2_ciclos_inativo++;
		}
		// Botao do pedestre semaforo 1
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == GPIO_PIN_SET){
			btn_s1++;
		}
		// Botao do pedestre semaforo 2
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == GPIO_PIN_SET){
			btn_s2++;
		}
    	//
    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);//USS1 trigger
    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//USS2 trigger*/
	}
}
void troca_semaforo(int semaforo){
	//Semaforo 1 vai para verde
	if(semaforo == 1){
		//5 segundos de amarelo
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); //S2Y
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); //S2G
		for(int i = 0; i < 5; i++){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); //P1R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //BUZ
			Delay_Us(500000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //BUZ
			Delay_Us(500000);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); //P1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //P1G

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET); //S2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET); //S2Y

		Delay_Us(2000000);

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); //S1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET); //S1G

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //P2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); //P2G

		//reseta contador do pedestre
		btn_s1 == 0;
	}
	else if(semaforo == 2){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); //S1Y
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
		for(int i = 0; i < 5; i++){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //BUZ
			Delay_Us(500000);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //P2R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //BUZ
			Delay_Us(500000);
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); //S2G
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //P2G

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); //S1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //S1Y

		Delay_Us(2000000);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET); //S2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); //S2G

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //P1G

		//reseta contador do pedestre
		btn_s2 == 0;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
