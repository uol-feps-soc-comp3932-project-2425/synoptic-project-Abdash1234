// filepath: /home/abdullah/esp/mqtt/main/mqtt_app.c
#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT";

// Updated MQTT event handler with the correct signature.
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(event->client, "epuck/cmd", 0);
            esp_mqtt_client_publish(event->client, "epuck/status", "Hello from e-puck!", 0, 1, 0);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("Topic: %.*s, Data: %.*s\n",
                   event->topic_len, event->topic,
                   event->data_len, event->data);
        
            // Check if the message is from the command topic
            if (strncmp(event->topic, "epuck/cmd", event->topic_len) == 0) {
                // Example: if the command is "move", then respond.
                if (strncmp(event->data, "move", event->data_len) == 0) {
                    ESP_LOGI(TAG, "Received move command, publishing response");
                    esp_mqtt_client_publish(event->client, "epuck/status", "Executing move command", 0, 1, 0);
                }
                // Additional command handling can be added here
            }
            break;
        
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
            
        default:
            break;
    }
}


void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtt://10.42.0.1:1883",
        },
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}