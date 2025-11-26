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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void iniciar_DWT(void)
{
	// Habilitar DWT
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// MEDICIÓN DE LOS PRINT (SE PASA EL NÚMERO DE VECES QUE SE QUIERE REALIZAR LA OPERACIÓN COMO PARÁMETRO)
void test_printeo_caracter(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;
    
    while (counter < iteraciones)
    {
        printf("a\n");
        counter++;
    }
}

void test_printeo_multiples_caracteres(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;
    
    while (counter < iteraciones)
    {
        printf("[TEST] Printf simple.\n");
        counter++;
    }
}

// MEDICIÓN DE LAS ASIGNACIONES SIMPLES
void test_asignaciones_8bits(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;

    volatile uint8_t asignacion;

    while (counter < iteraciones)
    {
        asignacion = rand() & 0xFF;
        counter++;
    }

}

void test_asignaciones_16bits(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;

    volatile uint16_t asignacion;

    while (counter < iteraciones)
    {
        asignacion = rand() & 0xFFFF;
        counter++;
    }

}

void test_asignaciones_32bits(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;

    volatile uint32_t asignacion;

    while (counter < iteraciones)
    {
        asignacion = ((uint32_t)rand() << 16) | (uint32_t)rand();
        counter++;
    }

}

// MEDICIÓN DE TIM2
void test_lectura_tim_dos(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;

    while (counter < iteraciones)
    {
        uint32_t t = TIM2->CNT;  // <--- lectura de TIM2
        counter++;
    }

}

// MEDICIÓN SWITCH CASE
void test_switch(uint32_t iteraciones)
{
    uint16_t GPIO_Pin = GPIO_PIN_3;
    uint8_t canal = 0;

    volatile uint32_t counter = 0;

    while (counter < iteraciones)
    {
        switch (GPIO_Pin) {
            case GPIO_PIN_1: canal = 1; break;
            case GPIO_PIN_2: canal = 2; break;
            case GPIO_PIN_3: canal = 3; break;
            case GPIO_PIN_4: canal = 4; break;
            default: canal = 99; break;
        }
        counter++;
    }
}

// MEDICIÓN DE LA CONDICIÓN IF, el segundo parámetro es el número de condiciones que se quiere probar
void test_if(uint32_t iteraciones, uint8_t condiciones)
{
    volatile uint32_t counter = 0;

    volatile uint32_t a = 0;
    volatile uint32_t b = 1;
    volatile uint32_t c = 1;

    if (condiciones == 1)
    {
        while (counter < iteraciones)
        {
            if (a > b) {
                continue;
            }
            counter++;
        }
    }
    else if (condiciones == 2)
    {
        while (counter < iteraciones)
        {
            if (a > b && c != a) {
                continue;
            }
            counter++;
        }
    }
    else if (condiciones == 3)
    {
        while (counter < iteraciones)
        {
            if (a > b && c != a || a == b) {
                continue;
            }
            counter++;
        }
    }
    else if (condiciones == 4)
    {
        while (counter < iteraciones)
        {
            if (a > b && c != a || a == b && c != b) {
                continue;
            }
            counter++;
        }
    }
}


void test_coincidencias(uint32_t iteraciones)
{
    volatile uint32_t counter = 0;

    uint32_t t1 = 1000;
    uint32_t t2 = 1200;
    uint8_t ch1 = 3;
    uint8_t ch2 = 4;

    while (counter < iteraciones)
    {
        if ( ch1 != ch2 &&
             (t2 - t1) >= 50 &&
             (t2 - t1) <= 800 ) {
            __NOP();
        }
        counter++;
    }

}


void medir_tiempo_ejemplo(void)
{

    uint32_t inicio, fin, ciclos, tiempo_ns;

    // 1) MEDIR TIEMPO DE UN printf()
    inicio = DWT->CYCCNT;

    test_printeo_caracter(100); // UN SOLO CARACTER
    //test_printeo_multiples_caracteres(100); // MÚLTIPLES CARACTERES

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo printf(): %lu ns\r\n\r\n", tiempo_ns);



    // 2) MEDIR UNA ASIGNACIÓN SIMPLE
    DWT->CYCCNT = 0;
    inicio = DWT->CYCCNT;

    test_asignaciones_8bits(100); // Probar asignaciones para variables de 8 bits
    //test_asignaciones_16bits(100); // Probar asignaciones para variables de 16 bits
    //test_asignaciones_32bits(100); // Probar asignaciones para variables de 32 bits

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo asignación simple: %lu ns\r\n\r\n", tiempo_ns);



    // 3) MEDIR LECTURA DE TIM2->CNT
    DWT->CYCCNT = 0;
    inicio = DWT->CYCCNT;

    test_lectura_tim_dos(100);

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo lectura TIM2->CNT: %lu ns\r\n\r\n", tiempo_ns);



    // 4) MEDIR EL switch
    DWT->CYCCNT = 0;
    inicio = DWT->CYCCNT;

    test_switch(100);

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo switch-case: %lu ns\r\n\r\n", tiempo_ns);

    // 5) MEDIR IF Statements con distinto número de condiciones
    DWT->CYCCNT = 0;
    inicio = DWT->CYCCNT;

    test_if(100, 3); // Se pueden probar condicionales de 1 a 4 (segundo parámetro)

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo condicional-if: %lu ns\r\n\r\n", tiempo_ns);


    // 6) MEDIR COMPARACIONES DE LAS COINCIDENCIAS
    DWT->CYCCNT = 0;
    inicio = DWT->CYCCNT;

    test_coincidencias(100);

    fin = DWT->CYCCNT;
    ciclos = fin - inicio;
    tiempo_ns = (ciclos * (1e9f / SystemCoreClock));
    printf("Tiempo bloque coincidencia (comparaciones): %lu ns\r\n\r\n", tiempo_ns);


    // RESUMEN
    printf("===========================================\r\n");
    printf(" Medición completada.\r\n");
    printf(" Frecuencia CPU: %lu Hz\r\n", SystemCoreClock);
    printf("===========================================\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  /* USER CODE BEGIN 2 */
  iniciar_DWT();
  printf("\n\n Inicio Medición\r\n");
  medir_tiempo_ejemplo();
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
