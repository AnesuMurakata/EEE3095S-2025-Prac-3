
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

// Task 4: Image splitting and processing variables
// Image dimensions for Task 4: 520x520, 1280x720, 1920x1080
// int task4_image_widths[3] = {520, 1280, 1920};
// int task4_image_heights[3] = {520, 720, 1080};

// Execution times for Task 4 (only wall clock time needed)
uint32_t task4_execution_times_fixed[5];
uint32_t task4_execution_times_double[5];

// Checksums for Task 4
uint64_t task4_checksums_fixed[5];
uint64_t task4_checksums_double[5];

// Chunking parameters
#define MAX_CHUNK_SIZE 64  // 64x64 chunks for safety with 4KB SRAM
#define CHUNK_OVERLAP 0    // No overlap needed for Mandelbrot

// Chunk processing counters
uint32_t total_chunks_processed = 0;
uint32_t current_chunk_x = 0;
uint32_t current_chunk_y = 0;

// Task 7: Fixed-point scaling factor testing
// Scaling factors to test: 10^3, 10^4, 10^6
int64_t scaling_factors[3] = {1000, 10000, 1000000};  // 10^3, 10^4, 10^6
char* scaling_names[3] = {"10^3", "10^4", "10^6"};

// Results for each scaling factor (15 total tests: 3 scaling factors × 5 image sizes)
uint32_t task7_execution_times[15];    // 15 total tests
uint64_t task7_checksums[15];          // 15 total tests
uint32_t task7_overflow_counts[15];    // 15 total tests

// Image sizes from Practical 1B (using current task3_image_sizes)
int task7_image_sizes[5] = {128, 160, 192, 224, 256};

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

// Task 4 function prototypes
void process_image_in_chunks(int full_width, int full_height, int max_iter, 
                            uint64_t (*mandelbrot_func)(int, int, int),
                            uint32_t *execution_time, uint64_t *total_checksum);
uint64_t calculate_mandelbrot_chunk(int chunk_x, int chunk_y, int chunk_width, int chunk_height,
                                   int full_width, int full_height, int max_iter,
                                   uint64_t (*mandelbrot_func)(int, int, int));
uint64_t calculate_single_pixel_mandelbrot_fixed(int x, int y, int width, int height, int max_iter);
uint64_t calculate_single_pixel_mandelbrot_double(int x, int y, int width, int height, int max_iter);
void reset_chunk_counters(void);

// Task 7 function prototypes
uint64_t calculate_mandelbrot_fixed_point_with_scale(int width, int height, int max_iter, int64_t scale_factor, uint32_t* overflow_count);
void test_scaling_factors(void);
void analyze_scaling_results(void);

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

// Task 4: Image chunking and processing functions

// Reset chunk processing counters
void reset_chunk_counters(void) {
    total_chunks_processed = 0;
    current_chunk_x = 0;
    current_chunk_y = 0;
}

// Calculate Mandelbrot for a single pixel using fixed-point arithmetic
uint64_t calculate_single_pixel_mandelbrot_fixed(int x, int y, int width, int height, int max_iter) {
    // Fixed-point scaling factor (10^6 as suggested in Appendix B)
    const int64_t SCALE = 1000000;
    
    // Pre-calculate scaled constants
    int64_t scaled_3_5 = 3 * SCALE + (5 * SCALE) / 10;  // 3.5 * SCALE
    int64_t scaled_2_0 = 2 * SCALE;                      // 2.0 * SCALE
    int64_t scaled_2_5 = 2 * SCALE + (5 * SCALE) / 10;  // 2.5 * SCALE
    int64_t scaled_1_0 = SCALE;                          // 1.0 * SCALE
    int64_t scaled_4 = 4 * SCALE;                        // 4 * SCALE
    int64_t scaled_2 = 2 * SCALE;                        // 2 * SCALE
    
    // Calculate x0 = (x / width) * 3.5 - 2.5
    int64_t x0 = ((x * scaled_3_5) / width) - scaled_2_5;
    
    // Calculate y0 = (y / height) * 2.0 - 1.0
    int64_t y0 = ((y * scaled_2_0) / height) - scaled_1_0;
    
    // Initialize iteration variables
    int64_t xi = 0;
    int64_t yi = 0;
    int iteration = 0;
    
    // Main iteration loop
    while (iteration < max_iter && ((xi * xi + yi * yi) <= scaled_4)) {
        // temp = xi^2 - yi^2
        int64_t temp = ((xi * xi) / SCALE) - ((yi * yi) / SCALE);
        
        // yi = 2 * xi * yi + y0
        yi = ((scaled_2 * xi * yi) / SCALE) + y0;
        
        // xi = temp + x0
        xi = temp + x0;
        
        iteration++;
    }
    
    return (uint64_t)iteration;
}

// Calculate Mandelbrot for a single pixel using double arithmetic
uint64_t calculate_single_pixel_mandelbrot_double(int x, int y, int width, int height, int max_iter) {
    // Calculate x0 = (x / width) * 3.5 - 2.5
    double x0 = ((double)x / width) * 3.5 - 2.5;
    
    // Calculate y0 = (y / height) * 2.0 - 1.0
    double y0 = ((double)y / height) * 2.0 - 1.0;
    
    // Initialize iteration variables
    double xi = 0.0;
    double yi = 0.0;
    int iteration = 0;
    
    // Main iteration loop
    while (iteration < max_iter && ((xi * xi + yi * yi) <= 4.0)) {
        // temp = xi^2 - yi^2
        double temp = xi * xi - yi * yi;
        
        // yi = 2 * xi * yi + y0
        yi = 2.0 * xi * yi + y0;
        
        // xi = temp + x0
        xi = temp + x0;
        
        iteration++;
    }
    
    return (uint64_t)iteration;
}

// Calculate Mandelbrot for a specific chunk of the full image
uint64_t calculate_mandelbrot_chunk(int chunk_x, int chunk_y, int chunk_width, int chunk_height,
                                   int full_width, int full_height, int max_iter,
                                   uint64_t (*mandelbrot_func)(int, int, int)) {
    
    uint64_t chunk_checksum = 0;
    
    // Process each pixel in the chunk
    for (int local_y = 0; local_y < chunk_height; local_y++) {
        for (int local_x = 0; local_x < chunk_width; local_x++) {
            
            // Convert local chunk coordinates to full image coordinates
            int global_x = chunk_x + local_x;
            int global_y = chunk_y + local_y;
            
            // Calculate Mandelbrot for this single pixel
            uint64_t pixel_checksum;
            if (mandelbrot_func == calculate_mandelbrot_fixed_point_arithmetic) {
                pixel_checksum = calculate_single_pixel_mandelbrot_fixed(
                    global_x, global_y, full_width, full_height, max_iter);
            } else {
                pixel_checksum = calculate_single_pixel_mandelbrot_double(
                    global_x, global_y, full_width, full_height, max_iter);
            }
            
            chunk_checksum += pixel_checksum;
        }
    }
    
    return chunk_checksum;
}

// Main function to process large images in chunks
void process_image_in_chunks(int full_width, int full_height, int max_iter, 
                            uint64_t (*mandelbrot_func)(int, int, int),
                            uint32_t *execution_time, uint64_t *total_checksum) {
    
    uint32_t start_time = HAL_GetTick();
    uint64_t cumulative_checksum = 0;
    reset_chunk_counters();
    
    // Calculate number of chunks needed
    int chunks_x = (full_width + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    int chunks_y = (full_height + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    
    // Process image in chunks
    for (int chunk_y = 0; chunk_y < chunks_y; chunk_y++) {
        for (int chunk_x = 0; chunk_x < chunks_x; chunk_x++) {
            
            // Calculate chunk dimensions
            int chunk_start_x = chunk_x * MAX_CHUNK_SIZE;
            int chunk_start_y = chunk_y * MAX_CHUNK_SIZE;
            int chunk_width = (chunk_start_x + MAX_CHUNK_SIZE > full_width) ? 
                             (full_width - chunk_start_x) : MAX_CHUNK_SIZE;
            int chunk_height = (chunk_start_y + MAX_CHUNK_SIZE > full_height) ? 
                              (full_height - chunk_start_y) : MAX_CHUNK_SIZE;
            
            // Process this chunk
            uint64_t chunk_checksum = calculate_mandelbrot_chunk(
                chunk_start_x, chunk_start_y, chunk_width, chunk_height,
                full_width, full_height, max_iter, mandelbrot_func);
            
            // Accumulate checksum
            cumulative_checksum += chunk_checksum;
            total_chunks_processed++;
            
            // Optional: LED indication for progress
            if (total_chunks_processed % 10 == 0) {
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); // Blink LED every 10 chunks
            }
        }
    }
    
    uint32_t end_time = HAL_GetTick();
    *execution_time = end_time - start_time;
    *total_checksum = cumulative_checksum;
}

// Task 7: Fixed-point scaling factor testing functions

// Calculate Mandelbrot with variable scaling factor
uint64_t calculate_mandelbrot_fixed_point_with_scale(int width, int height, int max_iter, int64_t scale_factor, uint32_t* overflow_count) {
    uint64_t mandelbrot_sum = 0;
    *overflow_count = 0;
    
    // Pre-calculate scaled constants using variable scale factor
    int64_t scaled_3_5 = 3 * scale_factor + (5 * scale_factor) / 10;  // 3.5 * SCALE
    int64_t scaled_2_0 = 2 * scale_factor;                            // 2.0 * SCALE
    int64_t scaled_2_5 = 2 * scale_factor + (5 * scale_factor) / 10;  // 2.5 * SCALE
    int64_t scaled_1_0 = scale_factor;                                 // 1.0 * SCALE
    int64_t scaled_4 = 4 * scale_factor;                              // 4 * SCALE
    int64_t scaled_2 = 2 * scale_factor;                              // 2 * SCALE
    
    // Loop through each pixel
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Calculate x0 = (x / width) * 3.5 - 2.5
            int64_t x0 = ((x * scaled_3_5) / width) - scaled_2_5;
            
            // Calculate y0 = (y / height) * 2.0 - 1.0
            int64_t y0 = ((y * scaled_2_0) / height) - scaled_1_0;
            
            // Initialize iteration variables
            int64_t xi = 0;
            int64_t yi = 0;
            int iteration = 0;
            
            // Main iteration loop with overflow detection
            while (iteration < max_iter && ((xi * xi + yi * yi) <= scaled_4)) {
                // Check for potential overflow before calculations
                if (xi > 0 && xi > (INT64_MAX / scale_factor)) {
                    (*overflow_count)++;
                    break;
                }
                if (yi > 0 && yi > (INT64_MAX / scale_factor)) {
                    (*overflow_count)++;
                    break;
                }
                
                // temp = xi^2 - yi^2
                int64_t temp = ((xi * xi) / scale_factor) - ((yi * yi) / scale_factor);
                
                // yi = 2 * xi * yi + y0
                yi = ((scaled_2 * xi * yi) / scale_factor) + y0;
                
                // xi = temp + x0
                xi = temp + x0;
                
                iteration++;
            }
            
            mandelbrot_sum += iteration;
        }
    }
    
    return mandelbrot_sum;
}

// Test all scaling factors
void test_scaling_factors(void) {
    for (int scale_idx = 0; scale_idx < 3; scale_idx++) {
        int64_t current_scale = scaling_factors[scale_idx];
        
        for (int size_idx = 0; size_idx < 5; size_idx++) {
            int current_size = task7_image_sizes[size_idx];
            
            // Convert 2D indices to 1D index
            int test_index = scale_idx * 5 + size_idx;
            
            // LED indication for progress
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            
            // Record start time
            uint32_t start_time = HAL_GetTick();
            
            // Test with current scaling factor
            uint32_t overflow_count = 0;
            task7_checksums[test_index] = calculate_mandelbrot_fixed_point_with_scale(
                current_size, current_size, MAX_ITER, current_scale, &overflow_count);
            
            // Record end time
            uint32_t end_time = HAL_GetTick();
            task7_execution_times[test_index] = end_time - start_time;
            task7_overflow_counts[test_index] = overflow_count;
            
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(200);
        }
    }
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
    // enable_dwt_cycle_counter();
    
    // Task 3: Extended execution time measurement
    // MAX_ITER = 100, Image sizes: 128, 160, 192, 224, 256
    /*
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
    */
    
    // Task 4: Direct fixed-point arithmetic processing (no chunking)
    // Process images: 128x128, 160x160, 192x192, 224x224, 256x256
    for (int task4_test = 0; task4_test < 5; task4_test++) {
        int current_size = task3_image_sizes[task4_test];
        
        // Test Fixed Point Arithmetic - Direct processing
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        
        // Record start time
        uint32_t start_time = HAL_GetTick();
        
        // Call Mandelbrot function directly
        task4_checksums_fixed[task4_test] = calculate_mandelbrot_fixed_point_arithmetic(
            current_size, current_size, MAX_ITER);
        
        // Record end time
        uint32_t end_time = HAL_GetTick();
        task4_execution_times_fixed[task4_test] = end_time - start_time;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(500);
    }
    
    // All Task 4 tests completed - turn on both LEDs to indicate completion
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    
    // Hold the LEDs on for 3s to show Task 4 completion
    HAL_Delay(3000);
    
    // Turn off the LEDs
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    
    // Task 7: Fixed-point scaling factor testing
    // Test scaling factors: 10^3, 10^4, 10^6
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // Indicate Task 7 start
    test_scaling_factors();
    analyze_scaling_results();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    
    // All Task 7 tests completed - turn on both LEDs to indicate completion
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    
    // Hold the LEDs on for 3s to show Task 7 completion
    HAL_Delay(3000);
    
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
