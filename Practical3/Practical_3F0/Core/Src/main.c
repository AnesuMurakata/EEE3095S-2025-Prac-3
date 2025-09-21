
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
#include <stdint.h>
#include "stm32f0xx.h"
#include "core_cm0.h"  // For DWT access
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time 
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/

// Global variables for Task 1
// Image dimensions for testing (square images)
#define IMAGE_128 128
#define IMAGE_160 160
#define IMAGE_192 192
#define IMAGE_224 224
#define IMAGE_256 256

// Global checksum variable to hold the checksum returned from the mandelbrot function
uint64_t global_checksum;

uint32_t execution_time;

// Global start_time variable
uint32_t start_time;

// Global end_time variable
uint32_t end_time;

// Arrays to store execution times for different MAX_ITER values
uint32_t execution_times_fixed[5];    // For fixed-point arithmetic
uint32_t execution_times_double[5];   // For double arithmetic

// Arrays to store checksums for different MAX_ITER values  
uint64_t checksums_fixed[5];          // For fixed-point arithmetic
uint64_t checksums_double[5];         // For double arithmetic

// Array to store the MAX_ITER values being tested
int max_iter_values[5] = {100, 250, 500, 750, 1000};

// Array to store image sizes being tested (using 128x128 for all tests)
int image_sizes[5] = { IMAGE_128, IMAGE_128, IMAGE_128, IMAGE_128, IMAGE_128};

// Task 3: Extended measurement variables
// Wall clock time (ms)
uint32_t wall_clock_fixed[5];
uint32_t wall_clock_double[5];

// CPU clock cycles
uint32_t cpu_cycles_fixed[5];
uint32_t cpu_cycles_double[5];

// Throughput (pixels per second)
float throughput_fixed[5];
float throughput_double[5];

// Image sizes for Task 3: 128, 160, 192, 224, 256
int task3_image_sizes[5] = {128, 160, 192, 224, 256};

// Helper variables for measurements
uint32_t tick_start, tick_end;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

// Task 3 helper functions
void enable_dwt_cycle_counter(void);
uint32_t get_system_clock_frequency(void);
uint32_t get_dwt_cycles(void);
float calculate_throughput(int width, int height, uint32_t wall_clock_ms);
void measure_execution_metrics(int width, int height, int max_iter, 
                              uint64_t (*mandelbrot_func)(int, int, int),
                              uint32_t *wall_clock, uint32_t *cpu_cycles, 
                              float *throughput, uint64_t *checksum);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Task 3: Cycle counter functions for STM32F0
// Note: STM32F0 doesn't have DWT, so we'll use SysTick for cycle counting
// We'll estimate cycles based on system clock and execution time

void enable_dwt_cycle_counter(void) {
    // For STM32F0, we don't have DWT, so we'll use SysTick
    // This is a placeholder - actual cycle counting will be done differently
}

// Function to get the actual system clock frequency
uint32_t get_system_clock_frequency(void) {
    // Get the actual system clock frequency from RCC
    return HAL_RCC_GetSysClockFreq();
}

// Function to get estimated cycle count based on system clock
uint32_t get_dwt_cycles(void) {
    // For STM32F0, we'll estimate cycles based on actual system clock
    uint32_t system_clock = get_system_clock_frequency();
    
    // Calculate cycles based on wall clock time
    uint32_t wall_time_ms = tick_end - tick_start;
    uint32_t estimated_cycles = (wall_time_ms * system_clock) / 1000;
    
    return estimated_cycles;
}

// Function to calculate throughput (pixels per second)
float calculate_throughput(int width, int height, uint32_t wall_clock_ms) {
    if (wall_clock_ms == 0) return 0.0f;
    float total_pixels = (float)(width * height);
    float time_seconds = (float)wall_clock_ms / 1000.0f;
    return total_pixels / time_seconds;
}

// Function to measure execution with all metrics
void measure_execution_metrics(int width, int height, int max_iter, 
                              uint64_t (*mandelbrot_func)(int, int, int),
                              uint32_t *wall_clock, uint32_t *cpu_cycles, 
                              float *throughput, uint64_t *checksum) {
    
    // Measure wall clock time
    tick_start = HAL_GetTick();
    
    // Execute Mandelbrot function
    *checksum = mandelbrot_func(width, height, max_iter);
    
    // Stop measurements
    tick_end = HAL_GetTick();
    
    // Calculate results
    *wall_clock = tick_end - tick_start;
    
    // For STM32F0, estimate CPU cycles based on wall clock time
    // This is an approximation since we don't have DWT
    *cpu_cycles = get_dwt_cycles();
    
    *throughput = calculate_throughput(width, height, *wall_clock);
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
    
    // Enable DWT cycle counter for Task 3
    enable_dwt_cycle_counter();
    
    // Task 3: Extended execution time measurement
    // MAX_ITER = 100, Image sizes: 128, 160, 192, 224, 256
    for (int test_size = 0; test_size < 5; test_size++) {
        int current_image_size = task3_image_sizes[test_size];
        
        // Test Fixed Point Arithmetic
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        
        measure_execution_metrics(current_image_size, current_image_size, MAX_ITER,
                                 calculate_mandelbrot_fixed_point_arithmetic,
                                 &wall_clock_fixed[test_size],
                                 &cpu_cycles_fixed[test_size],
                                 &throughput_fixed[test_size],
                                 &checksums_fixed[test_size]);
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(200);
        
        // Test Double Arithmetic
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        
        measure_execution_metrics(current_image_size, current_image_size, MAX_ITER,
                                 calculate_mandelbrot_double,
                                 &wall_clock_double[test_size],
                                 &cpu_cycles_double[test_size],
                                 &throughput_double[test_size],
                                 &checksums_double[test_size]);
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
    
    // All tests completed - turn on both LEDs to indicate completion
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    
    // Hold the LEDs on for 2s to show completion
    HAL_Delay(2000);
    
    // Turn off the LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  
    /* USER CODE END 2 */
  
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation

  // Fixed-point scaling factor (10^6 as suggested in Appendix B)
  const int64_t SCALE = 1000000;

  // Pre-calculate scaled constants
  int64_t scaled_3_5 = 3 * SCALE + (5 * SCALE) / 10;  // 3.5 * SCALE
  int64_t scaled_2_0 = 2 * SCALE;                      // 2.0 * SCALE
  int64_t scaled_2_5 = 2 * SCALE + (5 * SCALE) / 10;  // 2.5 * SCALE
  int64_t scaled_1_0 = SCALE;                          // 1.0 * SCALE
  int64_t scaled_4 = 4 * SCALE;                        // 4 * SCALE
  int64_t scaled_2 = 2 * SCALE;                        // 2 * SCALE

  // Loop through each pixel (y coordinate)
  for (int y = 0; y < height; y++) {
    // Loop through each pixel (x coordinate)
    for (int x = 0; x < width; x++) {
      // Calculate x0 = (x / width) * 3.5 - 2.5
      int64_t x0 = ((x * scaled_3_5) / width) - scaled_2_5;

      // Calculate y0 = (y / height) * 2.0 - 1.0
      int64_t y0 = ((y * scaled_2_0) / height) - scaled_1_0;

      // Initialize iteration variables
      int64_t xi = 0;
      int64_t yi = 0;
      int iteration = 0;

      // Main iteration loop
      while (iteration < max_iterations &&
         ((xi * xi + yi * yi) <= scaled_4)) {

        // temp = xi^2 - yi^2
        int64_t temp = ((xi * xi) / SCALE) - ((yi * yi) / SCALE);

        // yi = 2 * xi * yi + y0
        yi = ((scaled_2 * xi * yi) / SCALE) + y0;

        // xi = temp + x0
        xi = temp + x0;

        iteration++;
      }

      // Add iteration count to checksum
      mandelbrot_sum += iteration;
    }
  }


    
    return mandelbrot_sum;

}

//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation

        // Loop through each pixel (y coordinate)
        for (int y = 0; y < height; y++) {
            // Loop through each pixel (x coordinate)
            for (int x = 0; x < width; x++) {
                // Calculate x0 = (x / width) * 3.5 - 2.5
                double x0 = ((double)x / width) * 3.5 - 2.5;

                // Calculate y0 = (y / height) * 2.0 - 1.0
                double y0 = ((double)y / height) * 2.0 - 1.0;

                // Initialize iteration variables
                double xi = 0.0;
                double yi = 0.0;
                int iteration = 0;

                // Main iteration loop
                while (iteration < max_iterations &&
                       ((xi * xi + yi * yi) <= 4.0)) {

                    // temp = xi^2 - yi^2
                    double temp = xi * xi - yi * yi;

                    // yi = 2 * xi * yi + y0
                    yi = 2.0 * xi * yi + y0;

                    // xi = temp + x0
                    xi = temp + x0;

                    iteration++;
                }

                // Add iteration count to checksum
                mandelbrot_sum += iteration;
            }
        }

        return mandelbrot_sum;
    
    return mandelbrot_sum;
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
