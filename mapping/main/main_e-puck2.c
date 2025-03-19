

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
#include "esp_netif.h"
#include "esp_event.h"
#include "main_e-puck2.h"
#include "uart_e-puck2.h"
#include "rgb_led_e-puck2.h"
#include "button_e-puck2.h"
#include "mapping.h"
#include "mqtt_app.h"
#include "esp_wifi.h"
 
 #define IR_PROX_OFFSET  40    // Starting byte index for IR sensor data in the sensor packet
 #define NUM_IR_SENSORS  8     // e-puck2 has 8 IR sensors
 #define NUM_SAMPLES     3     // Number of samples to average for filtering

 #define EFFECTIVE_SPEED 1000   // 0.064 m/s ≈ 6.4 cm/s
 #define GRID_BUFFER_SIZE 1024
 #define PRINT_TAG  "GC-DEMO"
 #define WIFI_SSID "MyHotspot"
 #define WIFI_PASS "YourPassword"

 #define STATUS_TOPIC "epuck/status"
 #define SENSOR_TOPIC "epuck/sensor"
 #define MAP_TOPIC "epuck/map"
 #define CMD_TOPIC "epuck/cmd"
 #define THROUGHPUT_TOPIC "epuck/throughput"

 static const char *TAG = "wifi_station";

 // Event group to signal when connected
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

// Wi-Fi event handler
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI(TAG, "Disconnected from AP, retrying...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // IP_EVENT_STA_GOT_IP event data contains the IP address info
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create the default Wi-Fi station network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default configurations
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Create event group for Wi-Fi events
    wifi_event_group = xEventGroupCreate();

    // Register event handlers for Wi-Fi and IP events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    // Configure Wi-Fi connection settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            // Note: Setting the threshold for the auth mode is optional.
        },
    };

    // Set Wi-Fi mode to STA (station)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished. Waiting for connection...");

    // Wait until we are connected (CONNECTED_BIT is set)
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if(bits & CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", WIFI_SSID);
    }
}

 // Circular buffers to hold the last NUM_SAMPLES readings for each sensor.
 static uint16_t sensor_samples[NUM_IR_SENSORS][NUM_SAMPLES] = {0};
 static uint8_t sample_index[NUM_IR_SENSORS] = {0};

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

void straight_movement_task(int ms) {

	uart_get_data_ptr();
    // Move forward at a set speed.
    set_speed(1000);  // Adjust this value based on calibration.
    // printf("Moving straight for 2 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(ms));  // Move for 2 seconds.
    
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
// Convert the occupancy grid into a string.
void occupancy_grid_to_string(char *buffer, size_t buffer_size) {
    int offset = 0;
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            offset += snprintf(buffer + offset, buffer_size - offset, "%d ", occupancy_grid[i][j]);
            if (offset >= buffer_size) break;
        }
        offset += snprintf(buffer + offset, buffer_size - offset, "\n");
        if (offset >= buffer_size) break;
    }
}

// Snaking movement task: covers the designated area in a snake-like pattern.
void snake_movement_task(void *pvParameter) {
	char grid_str[GRID_BUFFER_SIZE];
    const int numRows = 5;      // Number of rows to cover; adjust as needed.
    int currentRow = 0;
    bool movingEast = true;     // Starting direction: assume robot initially faces east (right).

	// publish_with_timestamp("epuck/map", "Initial Grid");
	// occupancy_grid_to_string(grid_str, sizeof(grid_str));
	// publish_with_timestamp("epuck/map", grid_str);
    
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
	
	occupancy_grid_to_string(grid_str, sizeof(grid_str));
	publish_with_timestamp("epuck/map", grid_str);
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Add this function to your C code.
void throughput_task(void *pvParameter) {
    while (1) {
        // Publish a message on a dedicated topic.
        // You could use a fixed payload, or build one dynamically.
        publish_with_timestamp(THROUGHPUT_TOPIC, "Throughput test message from e-puck!");
        // Publish every 100 ms (adjust as needed).
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
 
 void app_main(void)
 {
	 // Initialize hardware (UART, LEDs, button, etc.)
	 uart_init();     // Sets up the UART to communicate with the STM32.
	 rgb_init();      // Optional: for LED feedback.
	 button_init();   // Optional: for reading the button state.
	 init_occupancy_grid(); // Initialize the occupancy grid.
	 wifi_init_sta();
     mqtt_app_start();

	 float data = 0.3;

	 // Update the occupancy grid with the simulated data.
	 xTaskCreatePinnedToCore(sensor_task, "sesnor_task", 2048, NULL, 4, NULL, 0);
	 xTaskCreatePinnedToCore(snake_movement_task, "snake_movement_task", 4096, NULL, 4, NULL, 1);
	 xTaskCreatePinnedToCore(throughput_task, "throughput_task", 2048, NULL, 4, NULL, 0);
	 update_occupancy_grid(data, 2);
	 print_occupancy_grid();
	 
	 publish_with_timestamp("epuck/status", "Hello from e-puck!");

  
	 // Main task can perform other operations or remain idle.
	 while(1) {
		  vTaskDelay(pdMS_TO_TICKS(1000));
	 }
 }