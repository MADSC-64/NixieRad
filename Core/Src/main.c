/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Statechart_required.h"
#include "Statechart.h"

#include "math.h"

#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// All segments ar PBs

#define A 4
#define B 1024
#define C 2048
#define D 4096
#define E 8192
#define F 32768
#define G 16384
#define DP 1

// All digits are PAs

#define DIG0 2
#define DIG1 128
#define DIG2 256
#define DIG3 16
#define DIG4 32
#define DIG5 64

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// 14-as timer'is tiksi kas 2 sekundes. Naudojamas periodiškam msV vidurkio atnaujinimui
// 16-as timer'is yra atsakingas už ekrano multipleksavimą
// 17-as timer'is yra atsakingas už klaviatūros multipleksavimą ir pa�?ios klaviatūros logiką

Statechart myStateChart;

//------------~~~~~~~~~~~~~~~~~-------------
// 				KLAVIATURA

char KeyPad_Symbols[4][4] = {
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
};

int CurrentRow = 0;
int PinWithInterruptNumber;

int ButtonWasPressed = 0;

char Numbers_Pressed[5] = {'\0','\0','\0','\0','\0'};
int Settings_Mode = 0;// kai bus 0 veiksime Fast/slow rezimuose
// 1 bus nustatomas msV, 2 bus nustatomas Threshold'as
int Number_Index = 0;
int Num_To_Show = 0; // cia setting'u rezime reikia kintamojo, kuris butu tuo momentu ivestas skaicius

char charPressed = '\0';

//------------~~~~~~~~~~~~~~~~~-------------
//				Ekranas

int PP_Sk = 0; //Praejusiu periodu skaicius
int Restart_PP_Sk = 6;

char Atvaizduojamas_Simbolis = 0;
int Atvaizduojami_Sk[5];
int kablelioVieta = 0; //Kabelio vieta gali būti prie 1-osios skilties (0.113), 2-osios, 3-iosios ir 4-osios
// Jei vaizduosime sveiką skaiciu, tai kablelio nedesime

int Numbers0_Text1 = 0; // kai turim Text rezima, ant ekrano yra piesiami tik 4 simboliai vietoje 5,
// nes tiek TSH, tiek COEF uzrasai uzima po 4 simbolius, ne 5

// Skaitiklio kintamieji

int Pulse_Count = 0;
double Pulse_Count_Average = 0;
double msV_Average = 0;
int Pulse_To_msV_Rate = 100000; // numatyta verte, kuria galima pakeisti
int msV_Threshold = 1;

int TIM_14_Periods_Passed = 0;
int Time_Passed = 0;

int FastMode_1_SlowMode_0 = 1;
int Pulse_Count_Array[5] = {0,0,0,0,0}; // Greitam rezimui
int Tick_Received = 0;

// Duomenu siuntimas per UART'a

char Message[7] = {' ',' ',' ',' ',' ','\n','\r'};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void IsvalytiIndikatoriu(void);
int SuformuotiSkaiciu(int sk);
void PavaizduotiSkaiciu(int sk, int skiltis, int taskas);
void ParodytiSkaiciu(double sk_double, char symbol);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  statechart_init(&myStateChart);
  HAL_TIM_Base_Start_IT(&htim17);
  statechart_enter(&myStateChart);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	  if(Tick_Received == 1 && Settings_Mode == 0){
		  if(FastMode_1_SlowMode_0 == 1){
				//Irasome nauja verte i masyva
				Pulse_Count_Array[(TIM_14_Periods_Passed-1) % 5] = Pulse_Count;
				// Per naujo skaiciuosime vidurki
				Pulse_Count_Average = 0;

				if(TIM_14_Periods_Passed < 5){

					for(int i=0;i<TIM_14_Periods_Passed;i++){
						Pulse_Count_Average+= Pulse_Count_Array[i];
					}

				}
				else{
					for(int i=0;i<5;i++){
						Pulse_Count_Average+= Pulse_Count_Array[i];
					}
				}
				Pulse_Count_Average = Pulse_Count_Average / ((double)Time_Passed/3600);
				Pulse_Count = 0;

			}
			if(FastMode_1_SlowMode_0 == 0){
				Pulse_Count_Average = (double)Pulse_Count/((double)Time_Passed/3600);
			}

			msV_Average = Pulse_Count_Average / Pulse_To_msV_Rate;

			if(msV_Average >= msV_Threshold){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); // ijungiame buzzer'i
			}
			else{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); // isjungiame buzzer'i
			}

			Tick_Received = 0;

			//Atvaizduojame duomenis

			if(msV_Average < 1){
				ParodytiSkaiciu(msV_Average*1000, 'n'); // Pi zymi micosV
			}
			else{
				ParodytiSkaiciu(msV_Average, '~'); // ~ zymi milisV
			}

			//Siunciame duomenis
			for(int i = 0;i<7;i++){
				Message[i] = ' ';
			}
			snprintf(Message, 7, "%d\n\r", (int)(msV_Average*1000));
			for(int i = 0;i<7;i++){
				if(Message[i] == '\0'){
					Message[i] = ' ';
				}
			}
			HAL_UART_Transmit(&huart2, (uint8_t*) Message, 7, HAL_MAX_DELAY);

	  }
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 10-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin PA6
                           PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB10 PB11
                           PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//--------------------------------------------------------
//					STATECHART'o funkcijos
//--------------------------------------------------------

void statechart_takeFastMeasurement( Statechart* handle){
	FastMode_1_SlowMode_0 = 1;
	Pulse_Count_Average = 0;
	Time_Passed = 0;
	TIM_14_Periods_Passed = 0;
	HAL_TIM_Base_Start_IT(&htim14);

}

void statechart_takeSlowMeasurement( Statechart* handle){
	FastMode_1_SlowMode_0 = 0;
	Pulse_Count_Average = 0;
	Time_Passed = 0;
	TIM_14_Periods_Passed = 0;
	HAL_TIM_Base_Start_IT(&htim14);

}
void statechart_changemsVRate( Statechart* handle){
	Settings_Mode = 1;
	Numbers0_Text1 = 1;
	// Sukonfiguruojame Atvaizduojami_Sk masyva, kad turetume uzrasa COEF
	Atvaizduojami_Sk[0] = 17;
	Atvaizduojami_Sk[1] = 0;
	Atvaizduojami_Sk[2] = 18;
	Atvaizduojami_Sk[3] = 19;

	//COEF 17, 0, 18,19
	//TSH 13,14,15,16
}
void statechart_changeThreshold( Statechart* handle){
	Settings_Mode = 2;
	Numbers0_Text1 = 1;
	// Sukonfiguruojame Atvaizduojami_Sk masyva, kad turetume uzrasa TSH
	Atvaizduojami_Sk[0] = 13;
	Atvaizduojami_Sk[1] = 14;
	Atvaizduojami_Sk[2] = 15;
	Atvaizduojami_Sk[3] = 16;
}

//--------------------------------------------------------
//					VFD valdymas
//--------------------------------------------------------

void IsvalytiIndikatoriu(void){

	// Valydami indikatorių visus PIN'us drive'insime HIGH (atidarys tranzus)

	HAL_GPIO_WritePin(GPIOB, 1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, 4, GPIO_PIN_SET);
	for(int i=pow(2,10);i<32769;i=i*2){
		HAL_GPIO_WritePin(GPIOB, i, GPIO_PIN_SET);
	}

	for(int i=2;i<257;i=i*2){
		HAL_GPIO_WritePin(GPIOA, i, GPIO_PIN_SET);
	}

}

int SuformuotiSkaiciu(int sk){

	//Apsirašysime kokius PB pin'us (segmentus) turime išjungti, kad gautume norimą skai�?ių, simbolį
	//TSH ir COEF
	int Atvaizdu_Lentele[20]={G,      		// 0
						  A|D|E|F|G,  		// 1
						  C|F,				// 2
						  E|F,				// 3
						  A|D|E,			// 4
						  B|E,				// 5
						  B,				// 6
						  D|E|F|G,			// 7
						  0,				// 8
						  E,				// 9
						  A|B|C|D|E|G,		// 10 Pi simbolis
						  A|B|C|D|E|F,		// 11 Brukšnys
						  A|B|C|D|E|F|G,	// 12 Bangelė
	  	  	  	  	  	  D|E|F|G,			// 13 T raidės kairysis brūkšnys ir "|"
						  B|C|D|E|F|G,		// 14 T dešinysis brūkšnys
						  B|E,        		// 15 S raidė
						  A|D,				// 16 H raidė
						  B|C|G,			// 17 C raidė
						  B|C,				// 18 E raidė
						  B|C|D};			// 19 F raidė
	return Atvaizdu_Lentele[sk];
}

void PavaizduotiSkaiciu(int sk, int skiltis, int taskas){

	//taskas = 1 –> yra taskas; taskas = bet kas kita –>nera tasko

	IsvalytiIndikatoriu();
	int Skiltys[6] = {DIG0,DIG1,DIG2,DIG3,DIG4,DIG5};

	int Segmentu_Kodas_PB = SuformuotiSkaiciu(sk);
	int Skilties_Kodas_PA = (DIG0|DIG1|DIG2|DIG3|DIG4|DIG5);
	Skilties_Kodas_PA -= Skiltys[skiltis];

	if (taskas != 1){
		Segmentu_Kodas_PB = Segmentu_Kodas_PB | DP;
	}

	GPIOB->ODR = Segmentu_Kodas_PB;
	GPIOA->ODR = Skilties_Kodas_PA;
}

void ParodytiSkaiciu(double sk_double, char symbol){

	//kablelioVieta = 1 reiškia, kad kablelis dedamas prie kairiausiojo skai�?iaus
	kablelioVieta = 0;
	Atvaizduojamas_Simbolis = symbol;
	//Išsiskaidome skai�?ių

	int sk = (int)(sk_double*10000);

	int desimtysTukstanciu = sk / 100000000;
	int tukstanciai = (sk % 100000000) / 10000000;
	int simtai = (sk % 10000000) / 1000000;
	int desimtys = (sk % 1000000) / 100000;
	int vienetai = (sk % 100000) / 10000;
	int desimtosios = (sk % 10000) / 1000;
	int simtosios = (sk % 1000) / 100;
	int tukstantosios = (sk % 100) / 10;
	int desimtaTukstantosios = (sk % 10) / 1;

	// Sumetame šį išskaidytą skai�?ių į masyvą, kad lengviau galėtume pasiekti norimus skai�?ius for kilpoje

	int skaitmenys[9] = {desimtysTukstanciu, tukstanciai, simtai, desimtys, vienetai, desimtosios, simtosios, tukstantosios, desimtaTukstantosios};

	int Atvaizduojamo_Sk_Indeksas = 0;

	for(int i = 0;i<9;i++){
		if(skaitmenys[i] != 0 && kablelioVieta == 0){
			//Radome pirma nenulini skaiciu
			if(i<4){
				kablelioVieta = 5-(i);
			}
		}
		if(kablelioVieta == 0 && i == 4){
			kablelioVieta = 1;
		}
		if(kablelioVieta !=0){
			Atvaizduojami_Sk[Atvaizduojamo_Sk_Indeksas] = skaitmenys[i];
			Atvaizduojamo_Sk_Indeksas++;
		}
		if(Atvaizduojamo_Sk_Indeksas>4){
			break;
		}
	}
    //Think about it for a second ok? šitas aukš�?iau parašytas alogritmas veikia taip:

    //1) Mes ieškome kur yra pirmasis nenulinis skai�?ius einant per masyvą iš kairės į dešinę.

    //2) Jei mes turime ne 0 tūkstan�?ių, tai mes norėsime parašyti visus 4 skai�?ius iki pat vienetų ir kablelio
    //   kaip ir neprireiks. Ta�?iau jei nerasime tukstan�?ių, bet rasime šimtus, norėsime parašyti vieną skai�?ių
    //   po kablelio. Šia logika judame taip toliau iki kol pamatome, kad neturime nei vieno sveiko skai�?iaus

    //3) Tai pamatę turime suprasti, kad reikės atvaizduoti skai�?ių 0.xyz. Vadinasi kablelio vieta turi būti
    //   prie pirmosios skilties

	// Reikėtų atkreipti dėmesį, jog yra trunc() versijos skai�?ius, ne round(), nes mes nieko neapvaliname

	// Pradedame 16-ą Timer'į
	MX_TIM16_Init();
	HAL_TIM_Base_Start_IT(&htim16);

}

//--------------------------------------------------------
//					Klaviatūros valdymas
//--------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == 256) { // PC8 skaitiklio isejimas
		Pulse_Count++;
	}

	if(ButtonWasPressed != 1){

		Numbers0_Text1 = 0;

		if (GPIO_Pin == 16) { // PC4 0 Column
			charPressed = KeyPad_Symbols[CurrentRow][0];
			PinWithInterruptNumber = 16;
		}
		if (GPIO_Pin == 32) { // PC5 1 Column
			charPressed = KeyPad_Symbols[CurrentRow][1];
			PinWithInterruptNumber = 32;
		}
		if (GPIO_Pin == 64) { // PC6 2 Column
			charPressed = KeyPad_Symbols[CurrentRow][2];
			PinWithInterruptNumber = 64;
		}
		if (GPIO_Pin == 128) { // PC7 3 Column
			charPressed = KeyPad_Symbols[CurrentRow][3];
			PinWithInterruptNumber = 128;
		}

		ButtonWasPressed = 1;

		switch(charPressed){
			case 'A':
				statechart_raise_beFast(&myStateChart);
				break;

			case 'B':
				statechart_raise_beSlow(&myStateChart);
				break;

			case 'C':
				statechart_raise_setmsVRate(&myStateChart);
				break;

			case 'D':
				statechart_raise_setThreshold(&myStateChart);
				break;
		}

		if(Settings_Mode >= 1 && charPressed <= 57 && charPressed >= 48){ // 48 – 0, o 57 – 9
			Numbers_Pressed[Number_Index] = charPressed;
			charPressed = '\0';
			Number_Index++;

			//Suformuojame skaiciu, kuri atvaizduosime
			Num_To_Show = 0;

			for(int i = 0;i<5;i++){
				if(Numbers_Pressed[i]<= 57 && Numbers_Pressed[i] >= 48){
					Num_To_Show = Num_To_Show*10 + (Numbers_Pressed[i]-48);
				}
			}
			ParodytiSkaiciu(Num_To_Show, 0); // 0 neturetu rodyti nei vieno simbolio
			if(Number_Index >= 5){
				for(int i = 0;i<5;i++){
					Numbers_Pressed[i] = '\0';
				}
				if(Settings_Mode == 1){
					Pulse_To_msV_Rate = Num_To_Show;
				}
				if(Settings_Mode == 2){
					 msV_Threshold = Num_To_Show;
				}
				Number_Index = 0;
				Settings_Mode = 0;
				statechart_raise_beFast(&myStateChart);
			}
		}
		if(Settings_Mode >=1 && charPressed =='#'){
			if(Settings_Mode == 1){
				Pulse_To_msV_Rate = Num_To_Show*1000;
			}
			if(Settings_Mode == 2){
				 msV_Threshold = Num_To_Show;
			}
			for(int i = 0;i<5;i++){
				Numbers_Pressed[i] = '\0';
			}
			Number_Index = 0;
			Settings_Mode = 0;
			statechart_raise_beFast(&myStateChart);
		}
	}
}

//--------------------------------------------------------
//					Timer'iai
//--------------------------------------------------------


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim14){
		TIM_14_Periods_Passed++;
		// Atnaujiname praejusi laika
		Time_Passed = TIM_14_Periods_Passed*2; // sekundemis
		Tick_Received = 1;
	}

	// Ekrano multipleksavimo timer'is
	if(htim == &htim16){
		if(PP_Sk == 1){
			//atvaizduojame pasirinktą simbolį
			if(Atvaizduojamas_Simbolis =='n'){
				PavaizduotiSkaiciu(10, 0, 0);
			}
			if(Atvaizduojamas_Simbolis =='-'){
				PavaizduotiSkaiciu(11, 0, 0);
			}
			if(Atvaizduojamas_Simbolis =='~'){
				PavaizduotiSkaiciu(12, 0, 1);
			}
		}
		else{
			if(PP_Sk > 0){
				if(kablelioVieta == PP_Sk-1){
					PavaizduotiSkaiciu(Atvaizduojami_Sk[PP_Sk-2], PP_Sk-1, 1);
				}
				else{
					PavaizduotiSkaiciu(Atvaizduojami_Sk[PP_Sk-2], PP_Sk-1, 0);
				}
			}
		}
		if((PP_Sk == Restart_PP_Sk && Numbers0_Text1 == 0)||(PP_Sk == Restart_PP_Sk-1 && Numbers0_Text1 == 1)){
			PP_Sk = 0;
		}
		else{
			PP_Sk++;
		}
	}

	if(htim == &htim17){
			if(ButtonWasPressed != 1){
				CurrentRow = (CurrentRow + 1) % 4;
				switch(CurrentRow){
				case 0:
					HAL_GPIO_WritePin(GPIOC, 8, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 1, GPIO_PIN_SET);
					break;
				case 1:
					HAL_GPIO_WritePin(GPIOC, 1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 2, GPIO_PIN_SET);
					break;
				case 2:
					HAL_GPIO_WritePin(GPIOC, 2, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 4, GPIO_PIN_SET);
					break;
				case 3:
					HAL_GPIO_WritePin(GPIOC, 4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, 8, GPIO_PIN_SET);
					break;
				}
			}
			else{
				if(HAL_GPIO_ReadPin(GPIOC, PinWithInterruptNumber) == 0){
					ButtonWasPressed = 0;
				}
			}
		}


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
