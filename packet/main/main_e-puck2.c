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
 
 #include "main_e-puck2.h"
 #include "uart_e-puck2.h"
 #include "rgb_led_e-puck2.h"
 #include "button_e-puck2.h"
 
 #define IR_PROX_OFFSET  40    // Starting byte index for IR sensor data in the sensor packet
 #define NUM_IR_SENSORS  8     // e-puck2 has 8 IR sensors
 #define NUM_SAMPLES     3     // Number of samples to average for filtering
 
 // Circular buffers to hold the last NUM_SAMPLES readings for each sensor.
 static uint16_t sensor_samples[NUM_IR_SENSORS][NUM_SAMPLES] = {0};
 static uint8_t sample_index[NUM_IR_SENSORS] = {0};
 
 /// Computes the moving average for a given sensor.
 /// The new reading is stored in a circular buffer and the average is computed.
 static uint16_t moving_average(uint16_t new_value, int sensor_index) {
	 sensor_samples[sensor_index][sample_index[sensor_index]] = new_value;
	 sample_index[sensor_index] = (sample_index[sensor_index] + 1) % NUM_SAMPLES;
	 
	 uint32_t sum = 0;
	 for (int j = 0; j < NUM_SAMPLES; j++) {
		 sum += sensor_samples[sensor_index][j];
	 }
	 return (uint16_t)(sum / NUM_SAMPLES);
 }
 
 static void sensor_task(void *pvParameter)
 {
	 while(1) {
		 // Fetch the next sensor packet (104 bytes expected).
		 sensors_buffer_t *sensor_buff = uart_get_data_ptr();
		 
		 printf("\r\nIR Sensor Values (Moving Average over %d samples):\r\n", NUM_SAMPLES);
		 for (int i = 0; i < NUM_IR_SENSORS; i++) {
			 // Combine two bytes to form an unsigned 16-bit integer.
			 uint16_t raw_value = sensor_buff->data[IR_PROX_OFFSET + 2*i] |
								  (sensor_buff->data[IR_PROX_OFFSET + 2*i + 1] << 8);
			 // Mask out the lower 12 bits to extract the actual IR sensor reading.
			 uint16_t ir_value = raw_value & 0x0FFF;
			 
			 // Compute the filtered value using the moving average filter.
			 uint16_t filtered_value = moving_average(ir_value, i);
			 
			 // Print both the raw and filtered sensor values.
			 printf("  Sensor %d: raw=0x%04X, IR value=%d, filtered=%d\r\n",
					i, raw_value, ir_value, filtered_value);
		 }
		 printf("Note: Higher filtered values indicate a closer or more reflective object.\r\n");
 
		 // Delay 1 second before the next reading.
		 vTaskDelay(pdMS_TO_TICKS(1000));
	 }
 }
 
 void app_main(void)
 {
	 // Initialize hardware (UART, LEDs, button, etc.)
	 uart_init();     // Sets up the UART to communicate with the STM32.
	 rgb_init();      // Optional: for LED feedback.
	 button_init();   // Optional: for reading the button state.
  
	 // Create the sensor reading/logging task.
	 xTaskCreate(
		  sensor_task,        // Task function.
		  "sensor_task",      // Task name.
		  2048,               // Stack size.
		  NULL,               // Task parameter.
		  4,                  // Task priority.
		  NULL                // Task handle.
	 );
  
	 // Main task can perform other operations or remain idle.
	 while(1) {
		  vTaskDelay(pdMS_TO_TICKS(1000));
	 }
 }
 