/*
 * Minimal example that reads IR proximity sensors from the STM32,
 * applies a simple moving average filter (over 3 samples), and logs
 * the values to the console via UART on the ESP32.
 *
 * The IR sensor readings are stored as 12-bit values within a 16-bit field.
 * This example uses a 3-sample moving average filter for each sensor.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <xtensa_api.h>
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "main_e-puck2.h"
#include "uart_e-puck2.h"
#include "rgb_led_e-puck2.h"
#include "button_e-puck2.h"
#include "mapping.h"
 
 #define IR_PROX_OFFSET  40    // Starting byte index for IR sensor data in the sensor packet
 #define NUM_IR_SENSORS  8     // e-puck2 has 8 IR sensors
 #define NUM_SAMPLES     3     // Number of samples to average for filtering

 #define EFFECTIVE_SPEED 1000   // 0.064 m/s ≈ 6.4 cm/s
 #define PRINT_TAG  "GC-DEMO"


// Your existing function that moves forward for a given time in milliseconds.
// This function sets the motor speed, delays for time_ms, then stops.
// void moveforward(uint32_t time_ms) {
// 	printf("Moving forward for %ld ms\n", time_ms);
//     set_speed(1000);  // set speed to 500 (adjust if needed)
//     vTaskDelay(time_ms / portTICK_PERIOD_MS);
//     // set_speed(0);    // stop the robot
// }

// New function: moves forward a specified distance (in meters)
// void move_forward_distance(float distance) {
//     // Calculate time in ms: time (s) = distance / speed, then convert to ms.
//     uint32_t time_ms = (uint32_t)((distance / EFFECTIVE_SPEED) * 1000.0);
//     printf("Moving forward %.2f m for %ld ms\n", distance, time_ms);
//     moveforward(time_ms);
// }
 
//  // Circular buffers to hold the last NUM_SAMPLES readings for each sensor.
//  static uint16_t sensor_samples[NUM_IR_SENSORS][NUM_SAMPLES] = {0};
//  static uint8_t sample_index[NUM_IR_SENSORS] = {0};
 
//  /// Computes the moving average for a given sensor.
//  /// The new reading is stored in a circular buffer and the average is computed.
//  static uint16_t moving_average(uint16_t new_value, int sensor_index) {
// 	 sensor_samples[sensor_index][sample_index[sensor_index]] = new_value;
// 	 sample_index[sensor_index] = (sample_index[sensor_index] + 1) % NUM_SAMPLES;
	 
// 	 uint32_t sum = 0;
// 	 for (int j = 0; j < NUM_SAMPLES; j++) {
// 		 sum += sensor_samples[sensor_index][j];
// 	 }
// 	 return (uint16_t)(sum / NUM_SAMPLES);
//  }
 
//  static void sensor_task(void *pvParameter)
// {
//     while(1) {
//         sensors_buffer_t *sensor_buff = uart_get_data_ptr();
//         printf("\r\nIR Sensor Values (Moving Average over %d samples):\r\n", NUM_SAMPLES);

//         // Let's assume sensor 0 is the front sensor.
//         uint16_t raw_value = sensor_buff->data[IR_PROX_OFFSET + 0] | 
//                              (sensor_buff->data[IR_PROX_OFFSET + 1] << 8);
//         uint16_t ir_value = raw_value & 0x0FFF;
//         uint16_t filtered_value = moving_average(ir_value, 0);

//         // Print sensor value.
//         printf("Sensor 0: raw=0x%04X, IR value=%d, filtered=%d\r\n",
//                raw_value, ir_value, filtered_value);

//         // Convert filtered IR value to a distance (meters).
//         // float distance = convert_ir_value_to_distance(filtered_value);
//         // printf("Converted distance: %.2f m\r\n", distance);

//         // Update the occupancy grid using the sensor reading.
//         // For example, update row 2 of the grid with this sensor reading.
//         // update_occupancy_grid(distance, 2);

//         // Optionally print the updated occupancy grid.
//         // print_occupancy_grid();

//         // Delay 1 second before the next reading.
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void straight_movement_task(int ms) {

    sensors_buffer_t *sensor_buff;


	uart_get_data_ptr();

    // Move forward at a set speed.
    set_speed(1000);  // Adjust this value based on calibration.
    // printf("Moving straight for 2 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(ms));  // Move for 2 seconds.
    
    // Stop the robot.
    // set_speed(0);
    printf("Stopped.\n");
    
}

// Parameter: direction: +1 for right turn, -1 for left turn.
void imu_turn_90(int direction) {
    // Validate parameter.
    if (direction != 1 && direction != -1) {
        ESP_LOGE(PRINT_TAG, "Invalid direction. Use 1 for right turn, -1 for left turn.");
        return;
    }
    
    // Calibration: compute gyro offset by averaging 64 samples.
    int32_t offset_sum = 0;
    uint8_t num_samples = 0;
    int16_t gyro_z;
    sensors_buffer_t *sensor_buff;
    while (num_samples < 64) {
        sensor_buff = uart_get_data_ptr();
        gyro_z = sensor_buff->data[22] + (sensor_buff->data[23] << 8);
        offset_sum += gyro_z;
        num_samples++;
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between samples
    }
    int16_t gyro_z_offset = offset_sum >> 6;
    ESP_LOGI(PRINT_TAG, "Gyro calibration offset: %d", gyro_z_offset);
    
    // Set pivot turn speed based on direction.
    // Here, a positive pivot turn speed is used for a right turn, and negative for left.
    if (direction == 1) {
        set_pivot_turn_speed(150);
    } else {
        set_pivot_turn_speed(-150);
    }
    
    // Initialize integration variables.
    float angle_deg = 0.0;
    float gyro_z_dps = 0.0;
    int64_t start, end;
    float time_interval_us;
    
    start = esp_timer_get_time();
    
    // Integrate gyro readings until the target turn angle is reached.
    // For a right turn (direction == 1), we expect negative angle accumulation.
    // For a left turn (direction == -1), we expect positive accumulation.
    while (1) {
        sensor_buff = uart_get_data_ptr();
        gyro_z = sensor_buff->data[22] + (sensor_buff->data[23] << 8);
        // Remove the offset:
        gyro_z -= gyro_z_offset;
        // Convert raw gyro reading to degrees per second.
        gyro_z_dps = ((float)gyro_z * 250.0f) / 32768.0f;
        
        end = esp_timer_get_time();
        time_interval_us = (float)(end - start);
        start = end;
        
        // Integrate to obtain the angle.
        angle_deg += gyro_z_dps * time_interval_us / 1000000.0f;
        
        // For a right turn, stop when the integrated angle is ≤ -90°.
        // For a left turn, stop when the integrated angle is ≥ 90°.
        if (direction == 1 && angle_deg <= -90.0f) {
            break;
        }
        if (direction == -1 && angle_deg >= 90.0f) {
            break;
        }
    }
    
    // Stop turning.
    set_speed(0);
    ESP_LOGI(PRINT_TAG, "Turn complete: angle reached %.2f degrees", angle_deg);
}

// Snaking movement task: covers the designated area in a snake-like pattern.
void snake_movement_task(void *pvParameter) {
    const int numRows = 5;      // Number of rows to cover; adjust as needed.
    int currentRow = 0;
    bool movingEast = true;     // Starting direction: assume robot initially faces east (right).
    
    while (currentRow < numRows) {
        // 1. Move straight along the current row.
        printf("Row %d: Moving straight...\n", currentRow);
        // Adjust the duration so the robot covers the row's length.
        straight_movement_task(2000);  // Move forward for 2000 ms; calibrate as needed.
        vTaskDelay(pdMS_TO_TICKS(500));  // Brief pause.
        
        // 2. Turn 90° to begin row transition.
        if (movingEast) {
            imu_turn_90(1);   // Turn right 90°.
        } else {
            imu_turn_90(-1);  // Turn left 90°.
        }
        vTaskDelay(pdMS_TO_TICKS(200));  // Short pause after turning.
        
        // 3. Move forward a short distance to shift to the next row.
        printf("Row %d: Shifting to next row...\n", currentRow);
        straight_movement_task(500);   // Move sideways for 500 ms; adjust for proper lateral shift.
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // 4. Turn 90° again to reorient for moving along the new row.
        if (movingEast) {
            imu_turn_90(1);   // Turn right to face west.
        } else {
            imu_turn_90(-1);  // Turn left to face east.
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // 5. Toggle horizontal direction and move to the next row.
        movingEast = !movingEast;
        currentRow++;
    }
    
    // When the snaking pattern is complete, stop the robot.
    set_speed(0);
    printf("Snaking pattern complete.\n");
    
    // Optionally, keep the task alive.
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


 
 void app_main(void)
 {
	 // Initialize hardware (UART, LEDs, button, etc.)
	 uart_init();     // Sets up the UART to communicate with the STM32.
	 rgb_init();      // Optional: for LED feedback.
	 button_init();   // Optional: for reading the button state.
	 init_occupancy_grid(); // Initialize the occupancy grid.

	 // Start moving forward (set_speed's parameter can be adjusted for speed).
	//  vTaskDelay(100/portTICK_PERIOD_MS);	
	//  imu_turn_90(1);
	//  vTaskDelay(100/portTICK_PERIOD_MS);
	//  imu_turn_90(-1);
	//  xTaskCreatePinnedToCore(straight_movement_task, "straight_movement_task", 2048, NULL, 4, NULL,1);
 
	 // Stop the bot.
	//  printf("Setting the speed to 0");
	//  set_speed(0);
	//  move_forward_distance(0.5);

	 float data = 0.3;

	 // Update the occupancy grid with the simulated data.
	 xTaskCreatePinnedToCore(snake_movement_task, "snake_movement_task", 4096, NULL, 4, NULL, 1);
	 update_occupancy_grid(data, 2);
	 print_occupancy_grid();

  
	//  // Create the sensor reading/logging task.
	//  xTaskCreate(
	// 	  sensor_task,        // Task function.
	// 	  "sensor_task",      // Task name.
	// 	  2048,               // Stack size.
	// 	  NULL,               // Task parameter.
	// 	  4,                  // Task priority.
	// 	  NULL                // Task handle.
	//  );
  
	 // Main task can perform other operations or remain idle.
	 while(1) {
		  vTaskDelay(pdMS_TO_TICKS(1000));
	 }
 }
 