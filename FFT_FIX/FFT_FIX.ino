#include "stm32f4xx_hal_adc.h"
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>

arduinoFFT FFT;
ADC_HandleTypeDef hadc1;
LiquidCrystal_I2C lcd(0x27, 16, 2);

uint32_t val;
float volt;
// uint8_t adc_conv_complete_flag = 0;

#define SAMPLES 512                //Must be a power of 2
// #define SAMPLING_FREQUENCY 834000  //Hz, must be less than 10000 due to ADC
#define SAMPLING_FREQUENCY 834000
unsigned int sampling_period_us;
unsigned long microseconds;
double peak;

double freq = 41000.0;
double debit = 0;
#define PI 3.14159265358979323846
#define DIAMETER 0.05  // Diameter pipa dalam meter (5 cm)
#define degree 90
#define vSoundF 1478.0  // water with 20C

double vReal[SAMPLES];  // Real part of FFT array
double vImag[SAMPLES];  // Imaginary part of FFT array

// volatile double adc_dma_result;

void setup() {
  // put your setup code here, to run once:
  // adc
  HAL_Init();
  
  // SystemClock_Config();
  MX_GPIO_Init();
  // MX_DMA_Init();
  MX_ADC1_Init();
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  // sampling_period_us = 1;
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)vReal, SAMPLES);
  lcd.init();
  lcd.backlight();
}

void loop() {
  /* SAMPLING */
  // val = HAL_ADC_GetValue(&hadc1);
  // VOLTS = (ADC_COUNT * VREF) / (2 ^ ADC_NUMBER_OF_BITS)

  microseconds = micros();  // Overflows after around 70 minutes!
  for (int i = 1; i < SAMPLES; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    analogReadResolution(12);
    vReal[i] = (analogRead(PA0)* 3.3) / 4096.0;
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us) {
      //empty loop
    }
    microseconds += sampling_period_us;
    // Serial.println(vReal[i]);
  }
  // Serial.println("done!");

  //  Start ADC conversion using DMA
  // if(adc_conv_complete_flag == 1){
  //   for(int x = 1; x<SAMPLES; x++){
  //     Serial.println(vReal[x]);
  //   }
  //   adc_conv_complete_flag = 0;
  // }
  // Wait for DMA transfer to complete


  
  // /* FFT */
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  // Serial.println(volt);
  // Serial.print(peak);
  Serial.println(peak);

  // debit = calculateFlowRate(peak, freq, degree);
  // Serial.print("  ");
  // Serial.print(debit/1000,6);
  // Serial.println();


  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Freq : " + String(peak) + "KHz");
  // lcd.setCursor(0,1);
  // lcd.print("Deb: " + String(debit/1000,6) + "m^3");  
}

static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = { 0 };

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  // hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

double calculateFlowRate(double receivedFrequency, double transmissionFrequency, double relativeAngle) {
  // Hitung kecepatan aliran menggunakan rumus efek Doppler
  double velocity = (vSoundF * (receivedFrequency - transmissionFrequency)) / (2 * transmissionFrequency * cos(relativeAngle));
  // Hitung luas penampang pipa
  double crossSectionArea = PI * pow((DIAMETER / 2.0), 2);
  // Hitung debit air
  double flowRate = crossSectionArea * velocity;
  // if(receivedFrequency == (receivedFrequency+50.00) || receivedFrequency == (peak-50.00) || receivedFrequency == transmissionFrequency){
  //   flowRate = 0.0;
  // }
  // if (receivedFrequency == 0 || receivedFrequency == transmissionFrequency) {
  //   flowRate = 0.0;
  // }
  return fabs(flowRate+1);
}

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
// 	// I set adc_conv_complete_flag variable to 1 when,
// 	// HAL_ADC_ConvCpltCallback function is call.
// 	adc_conv_complete_flag = 1;
// }

// static void MX_DMA_Init(void)
// {

//   /* DMA controller clock enable */
//   __HAL_RCC_DMA2_CLK_ENABLE();

//   /* DMA interrupt init */
//   /* DMA2_Stream0_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

// }

// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//   RCC_OscInitStruct.PLL.PLLM = 8;
//   RCC_OscInitStruct.PLL.PLLN = 80;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//   RCC_OscInitStruct.PLL.PLLQ = 4;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }