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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
long uss1_delay = 0;
long uss2_delay = 0;
//Fluxo nas vias
long uss1_fluxo = 0; //semaforo 1
long uss2_fluxo = 0; // semafor 2
//distancia padrao sem carro
int uss1_calibragem = 1000;
int uss2_calibragem = 700;
//informa se carro ja passou pelo sensor
int uss1_resetado = 1;
int uss2_resetado = 1;
//contador do botao de pedestre
int btn_s1 = 0;
int btn_s2 = 0;
//contador de ciclos inativo
long uss1_ciclos_inativo = 0;
long uss2_ciclos_inativo = 0;
int uss1_seg_inativo = 0;
int uss2_seg_inativo = 0;
//amostragem por segundo
int tm_frequencia = 10;
//contagem de tempo
int count_time = 0;
int count_time_seg = 0;
//semaforo fechado 1 = s1 e 2 = s2
int sem_fechado = 1;
/* USER CODE END PV */
//semaforo que deve ser fechado
int fecha_sem = 1;

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
  MX_TIM3_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  //timer com interrupção externa para medir largura do echo dos sensores com resolução de 1us
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //uss2 descida do sinal de echo
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //uss2 subida do sinal de echo
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); //uss1 subida do sinal de echo
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); //uss1 descida do sinal de echo
  //timer com interrupação por tempo de ativação dos sensores (trigger). Resolução 100ms
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	//semaforo 1 fechado
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); //S1R
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //S1Y
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //P1G
	//semaforo 1 fechado
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET); //S2R
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET); //S2Y
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); //S2G
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //P2G
  while (1)
  {
	  //faz a troca dos semaforos
	  if(sem_fechado != fecha_sem){
		  fecha_semaforo(fecha_sem);
		  count_time = 0;
	  }
	  //Breakpoint "%d - USS1 d-%d f-%d b-%d - USS2 d-%d f-%d b-%d \n",count_time,uss1_delay,uss1_fluxo,btn_s1,uss2_delay,uss2_fluxo,btn_s2
	  //"%d USS1 I-%d b-%d USS2 I-%d b-%d - fechado %d - deve_fechar %d \n",count_time,uss1_ciclos_inativo,btn_s1,uss2_ciclos_inativo,btn_s2,sem_fechado,fecha_sem
	  HAL_Delay(500);
  /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2){
		//subida echo uss1
		if(htim->Channel==TIM_CHANNEL_3){

		}
		//descida echo uss1
		if(htim->Channel==TIM_CHANNEL_4){

		}
		//subida echo uss2
		if(htim->Channel==TIM_CHANNEL_2){

		}
		//descida echo uss2
		if(htim->Channel==TIM_CHANNEL_1){

		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM3) //Interrupção por tempo, 100ms
    {
    	count_time_seg = count_time/tm_frequencia;
    	//Calcula a largura do pulso de echo do sensor 1
    	uss1_delay=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4) - HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
    	//regulagem contra ruido
    	if(uss1_delay > 0 && uss1_delay < 1400){
    		//verifica se tem carro atravez da calibragem(delay normal sem carro)
			if(uss1_delay < uss1_calibragem){
				if(uss1_resetado == 1){
					uss1_fluxo++;
				}
				//informa que o sensor ainda nao voltou para distancia padrão
				uss1_resetado = 0;
				uss1_ciclos_inativo = 0;
			}
			else{
				//informa que o sensor voltou para distancia padrão
				uss1_resetado = 1;
				//contador de ciclos inativos incrementado
				uss1_ciclos_inativo++;
			}
    	}
    	//Calcula a largura do pulso de echo do sensor 2
    	uss2_delay=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1) - HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
    	//regulagem contra ruido
    	if(uss2_delay > 0 && uss2_delay < 900){
    		//verifica se tem carro atravez da calibragem(delay normal sem carro)
			if(uss2_delay < uss2_calibragem){
				if(uss2_resetado == 1){
					uss2_fluxo++;
				}
				//informa que o sensor ainda nao voltou para distancia padrão
				uss2_resetado = 0;
				uss2_ciclos_inativo = 0;
			}
			else{
				//informa que o sensor voltou para distancia padrão
				uss2_resetado = 1;
				//contador de ciclos inativos incrementado
				uss2_ciclos_inativo++;
			}
    	}
    	//Tempo de inatividade do sensor 1 em segundos
    	uss1_seg_inativo = uss1_ciclos_inativo/tm_frequencia;
    	//Tempo de inatividade do sensor 1 em segundos
    	uss2_seg_inativo = uss2_ciclos_inativo/tm_frequencia;

    	//Botao de pedestre do semaforo 1 foi pressionado
		if(btn_s1 > 0){
			//semaforo 1 esta fechado, ou seja, sinal de pedestre do semaforo 2
			//esta fechado.
			if(sem_fechado == 2){
				//Sensor do semaforo 1 nao detecta carro a mais de 5 segundos
				//entao fecha o semaforo 2 para travessia de pedestre.
				if(uss1_seg_inativo >= 5){
					//A condição btn_s2 == 0 faz com que haja tempo minimo para travessia
					//dos pedestres do semaforo 2.
					if(btn_s2 == 0){
						fecha_sem = 1;
					}
					//Tempo maximo de espera 15 segundos
					else if(count_time_seg >= 15){
						fecha_sem = 1;
					}
				}
			}
		}
		//Se botão de pedestre nao tiver sido pressionado, entao ler a entrada.
		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)){
			btn_s1++;
		}

    	//Botao de pedestre do semaforo 2 foi pressionado
    	if(btn_s2 > 0){
    		//semaforo 1 esta fechado, ou seja, sinal de pedestre do semaforo 2
    		//esta fechado.
    		if(sem_fechado == 1){
    			//Sensor do semaforo 2 nao detecta carro a mais de 5 segundos
    			//entao fecha o semaforo 2 para travessia de pedestre.
				if(uss2_seg_inativo >= 5){
					//A condição btn_s1 == 0 faz com que haja tempo minimo para travessia
					//dos pedestres do semaforo 1.
					if(btn_s1 == 0){
						fecha_sem = 2;
					}
					//Tempo maximo de espera 15 segundos
					else if(count_time_seg >= 15){
						fecha_sem = 2;
					}
				}
			}
		}
    	//Se botão de pedestre nao tiver sido pressionado, entao ler a entrada.
    	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)){
			btn_s2++;
		}
    	//Tempo máximo de um semaforo aberto 75seg(3 cliclos comuns de 25seg)
		if(count_time_seg >= 75){
			//Semaforo 1 fechado então fecha semaforo 2
			if(sem_fechado == 1){
				fecha_sem = 2;
			}
			//Semaforo 2 fechado então fecha semaforo 1
			else{
				fecha_sem = 1;
			}
		}
    	//tempo normal de semaforo atingido 25seg
		else if(count_time_seg >= 25){
    		//Semaforo 1 fechado e seu senssor detectou carros menos de 5 seg
    		//Ou botão do semaforo 2 foi pressionado.
    		if((sem_fechado == 1 && uss1_seg_inativo <= 5) || btn_s2 > 0){
				fecha_sem = 2;
			}
    		//Semaforo 2 fechado e seu senssor detectou carros menos de 5 seg
			//Ou botão do semaforo 1 foi pressionado.
    		else if((sem_fechado == 2 && uss2_seg_inativo <= 5) || btn_s1 > 0){
				fecha_sem = 1;
			}
    	}

    	//verifica inatividade no semaforo aberto
    	if(sem_fechado == 1 && uss1_seg_inativo <= 5 && uss2_seg_inativo > 5 && count_time_seg >5 && (btn_s1 == 0 || count_time_seg > 150)){
    		fecha_sem = 2;
    	}
    	else if(sem_fechado == 2 && uss2_seg_inativo <= 5 && uss1_seg_inativo > 5 && count_time_seg >5 && (btn_s2 == 0 || count_time_seg > 150)){
    		fecha_sem = 1;
    	}
    	//contador de ciclos
    	count_time++;

    	//Envia pulso de trigger para os sensores
    	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //USS1 trigger
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET); //USS2 trigger
		//delay de ativação
		Delay_Us(10);
		//Interrompe o pulso de trigger para os sensores
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//USS1 trigger
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);//USS2 trigger*/
		//Reinicia o contador interno do timer 2
		__HAL_TIM_SET_COUNTER(&htim2,0);
    }
}
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
void fecha_semaforo(int semaforo){
	if(semaforo == 1){
		//5 segundos de transição do verde para vermelho
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET); //S1Y
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
		//Sinal de aviso(visual e sonoro) para pedestres
		for(int i = 0; i < 5; i++){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //BUZ
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //P2R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //BUZ
			HAL_Delay(500);
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); //S2G
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); //P2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //P2G
		//reseta contador de pedestre do semaforo que vai abrir(semaforo 2)
		btn_s2 = 0;

		//Fecha semaforo 1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET); //S1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET); //S1Y

		//delay com ambos os semaforos em vermelho
		HAL_Delay(1500);

		//Abre o semaforo 2
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET); //S2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); //S2G

		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //P1G
		sem_fechado = 1;
	}
	else if(semaforo == 2){
		//5 segundos de transição do verde para vermelho
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); //S2Y
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET); //S2G
		//Sinal de aviso(visual e sonoro) para pedestres
		for(int i = 0; i < 5; i++){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); //P1R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); //BUZ
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET); //P1R
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET); //BUZ
			HAL_Delay(500);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET); //S1G
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET); //P1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //P1G
		//reseta contador de pedestre do semaforo que vai abrir(semaforo 1)
		btn_s1 = 0;

		//Fecha semaforo 2
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET); //S2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET); //S2Y

		//delay com ambos os semaforos em vermelho
		HAL_Delay(1500);

		//Abre semaforo 1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET); //S1R
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET); //S1G

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); //P2R
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); //P2G
		sem_fechado = 2;
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
