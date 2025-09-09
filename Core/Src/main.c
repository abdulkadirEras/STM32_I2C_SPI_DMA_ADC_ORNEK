/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *st7798 de spi veri göndermede problem olursa error_handler a gidecek
  *watch dog eklendi 500ms
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include "fonts.h"
#include "st7789.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t MPU6050;

typedef struct
{

	float xDerece;
	float yDerece;
	float sicaklik;

	float xOrtalama;
	float yOrtalama;
	float sicaklikOrtalama;

	char xDereceString[14];
	char yDereceString[14];
	char sicaklikString[14];

	uint8_t mpuKontrol;


}mpu6050;

typedef struct
{
	uint32_t zamanlayici;
	uint16_t sayac;
}program;


typedef struct
{
	uint16_t i, ADC_Sonuc[2];
	uint16_t adc1;
	uint16_t adc2;

	char adc1String[14];
	char adc2String[14];

}adcDegiskenleri;


ADC_ChannelConfTypeDef ADC_CH_Cfg = {0};
uint32_t ADC_Kanallari[2] = {ADC_CHANNEL_0,ADC_CHANNEL_1};
uint32_t ADC_RANK[2] = {ADC_REGULAR_RANK_1,ADC_REGULAR_RANK_2};
adcDegiskenleri adc;
mpu6050 mpuDegiskenleri;
program programDegiskenleri;


//uint64_t TxpipeAddrs = 0x11223344AA;
//char kablosuzCanVerisi[32] = "Hello World!";
//char AckPayload[32];

typedef struct
{
	uint64_t TxYolAdresi;

	char kablosuzVeri[32];
	char AckPayload[32];


}nrf24l01_Degiskenler;

nrf24l01_Degiskenler rfDegiskenler;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define durumMakSure 1
#define ortalamaSayisi 100


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void mpuOku(void);
void mpuHesapla(void);
void tftEkranBaslangic(void);
void degiskenDegerSifirla(void);
void tftEkranaYaz(void);
void sistemiAyarla(void);
void adcOku(void);


void uart_ile_gonder(char yazilan[])
{
	HAL_UART_Transmit (&huart1,(uint8_t *) yazilan, strlen(yazilan), 10);
}




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */





  //DMA İLE ÇALIŞIYOR.
  ST7789_Init();//Burada DMA ile ÇALIŞIYOR

  tftEkranBaslangic();



  HAL_IWDG_Refresh(&hiwdg);
  sistemiAyarla();
  HAL_IWDG_Refresh(&hiwdg);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GetTick()-programDegiskenleri.zamanlayici>=durumMakSure)
	  {
		  HAL_IWDG_Refresh(&hiwdg);
		  mpuOku();
		  adcOku();



		  programDegiskenleri.sayac++;
		  programDegiskenleri.zamanlayici=HAL_GetTick();
	  }


	  if(programDegiskenleri.sayac>=ortalamaSayisi)
	  {



		  mpuHesapla();

		  tftEkranaYaz();

		  degiskenDegerSifirla();

	  	  HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);

	  	  programDegiskenleri.sayac=0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void mpuOku(void)
{
	MPU6050_Read_All(&hi2c1, &MPU6050);

	mpuDegiskenleri.xOrtalama+=MPU6050.KalmanAngleX;
	mpuDegiskenleri.yOrtalama+=MPU6050.KalmanAngleY;
	mpuDegiskenleri.sicaklikOrtalama+=MPU6050.Temperature;
}


void mpuHesapla(void)
{
	mpuDegiskenleri.xDerece=mpuDegiskenleri.xOrtalama/ortalamaSayisi;
	mpuDegiskenleri.yDerece=mpuDegiskenleri.yOrtalama/ortalamaSayisi;
	mpuDegiskenleri.sicaklik=mpuDegiskenleri.sicaklikOrtalama/ortalamaSayisi;
}

void tftEkranBaslangic(void)
{
	//ST7789_Test();//resim felan da çizdirdiği için hafıza hatası veriyor diğer şeyleri kapat bunu açtığında
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(RED);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(BLUE);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(GREEN);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(YELLOW);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(BROWN);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(DARKBLUE);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(MAGENTA);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(LIGHTGREEN);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(LGRAY);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(LBBLUE);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_Fill_Color(WHITE);
	HAL_Delay(100);
	HAL_IWDG_Refresh(&hiwdg);
	ST7789_WriteString(20, 120, "Bikopter Kumanda", Font_11x18, RED, WHITE);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);

}

void sistemiAyarla(void)
{
	 mpuDegiskenleri.mpuKontrol=1;
	  while (mpuDegiskenleri.mpuKontrol == 1)
	  {
		  mpuDegiskenleri.mpuKontrol=MPU6050_Init(&hi2c1);

		  if(mpuDegiskenleri.mpuKontrol==1)
		  {
			  ST7789_Fill_Color(RED);
			  ST7789_WriteString(0, 120, "MPU6050 INIT  HATA!!!", Font_16x26, WHITE, RED);
		  }
		  else
		  {
			  ST7789_Fill_Color(GREEN);
			  HAL_Delay(100);
			  ST7789_WriteString(0, 120, "MPU6050 INIT  edildi", Font_16x26, WHITE,RED);
		  }
		  HAL_Delay(100);
	  }




	  adc.i=0;
	  adc.ADC_Sonuc[0]=0;
	  adc.ADC_Sonuc[1]=0;
	  adc.ADC_Sonuc[2]=0;
	  adc.ADC_Sonuc[3]=0;

	  mpuDegiskenleri.xOrtalama=0.0;
	  mpuDegiskenleri.yOrtalama=0.0;


	  mpuDegiskenleri.xDerece=0.0;
	  mpuDegiskenleri.yDerece=0.0;
	  mpuDegiskenleri.sicaklik=0.0;

	  programDegiskenleri.sayac=0;
	  programDegiskenleri.zamanlayici=HAL_GetTick();

}

void adcOku(void)
{
	for(adc.i=0; adc.i<2; adc.i++)
	{
	   	 ADC_CH_Cfg.Rank =  ADC_RANK[adc.i];
	   	 ADC_CH_Cfg.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	     ADC_CH_Cfg.Channel = ADC_Kanallari[adc.i];        // ADC Channel [i] seçilir
	     HAL_ADC_ConfigChannel(&hadc1, &ADC_CH_Cfg); // Seçilen ADC Channel ayarları yapılır
	     HAL_ADC_Start(&hadc1);                         // ADC çevrimi ve seçilen kanalın ayarları yapılır
	     HAL_ADC_PollForConversion(&hadc1, 1);         // Poll The ADC Channel With TimeOut = 1mSec
	     adc.ADC_Sonuc[adc.i] = HAL_ADC_GetValue(&hadc1);         // ADC çevrim sonucu okunur.
	 }
	 adc.adc1=adc.ADC_Sonuc[0];
	 adc.adc2=adc.ADC_Sonuc[1];

}

void tftEkranaYaz(void)
{
	 ST7789_Fill_Color(WHITE);
	 sprintf(mpuDegiskenleri.xDereceString,"X Der:%.2f",mpuDegiskenleri.xDerece);
	 ST7789_WriteString(0, 12, mpuDegiskenleri.xDereceString, Font_16x26, BLUE, GREEN);

	 sprintf(mpuDegiskenleri.yDereceString,"Y Der:%.2f",mpuDegiskenleri.yDerece);
	 ST7789_WriteString(0, 50, mpuDegiskenleri.yDereceString, Font_16x26, BLUE, GREEN);

	 sprintf(mpuDegiskenleri.sicaklikString,"Temp:%.2f",mpuDegiskenleri.sicaklik);
	 ST7789_WriteString(0, 88, mpuDegiskenleri.sicaklikString, Font_16x26, BLUE, GREEN);

	 sprintf(adc.adc1String,"ADC1:%d",adc.adc1);
	 ST7789_WriteString(0, 126, adc.adc1String, Font_16x26, BLUE, GREEN);

	 sprintf(adc.adc2String,"ADC2:%d",adc.adc2);
	 ST7789_WriteString(0, 164, adc.adc2String, Font_16x26, BLUE, GREEN);
}

void degiskenDegerSifirla(void)
{
	mpuDegiskenleri.xOrtalama=0.0;
	mpuDegiskenleri.yOrtalama=0.0;
	mpuDegiskenleri.sicaklikOrtalama=0.0;

	for (uint8_t i = 0; i < sizeof(mpuDegiskenleri.sicaklikString); ++i)
	{
		mpuDegiskenleri.sicaklikString[i]=0;
		mpuDegiskenleri.xDereceString[i]=0;
		mpuDegiskenleri.yDereceString[i]=0;

		adc.adc1String[i]=0;
		adc.adc2String[i]=0;

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
	  HAL_IWDG_Refresh(&hiwdg);
	  HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
	  HAL_Delay(400);


  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
