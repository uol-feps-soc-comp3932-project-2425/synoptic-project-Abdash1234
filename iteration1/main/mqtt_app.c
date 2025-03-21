#include "mqtt_client.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"
#include <sys/time.h>  

static const char *TAG = "MQTT";

// Make the MQTT client global.
static esp_mqtt_client_handle_t mqtt_client = NULL;
SemaphoreHandle_t mqtt_connected_sem = NULL;
uint32_t messages_published = 0;
uint32_t bytes_published = 0;
uint32_t publish_errors = 0;
uint32_t msg_seq = 0;

void mqtt_publish(const char *topic, const char *data)
{
    if (mqtt_client) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, 0);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Publish error on topic %s", topic);
            publish_errors++;
        } else {
            messages_published++;
            bytes_published += strlen(data);
        }
    } else {
        ESP_LOGE(TAG, "MQTT client not initialized");
        publish_errors++;
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
        
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xSemaphoreGive(mqtt_connected_sem);
            esp_mqtt_client_publish(event->client, "epuck/status", "New Iteration started", 0, 1, 0);
            mqtt_publish("epuck/status", "New Iteration Starterd [mqttPublish]");
            esp_mqtt_client_subscribe(event->client, "epuck/cmd", 0);
            // esp_mqtt_client_publish(event->client, "epuck/status", "Hello from e-puck! [MQTT FUNC]", 0, 1, 0);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("Topic: %.*s, Data: %.*s\n",
                   event->topic_len, event->topic,
                   event->data_len, event->data);
            if (strncmp(event->topic, "epuck/cmd", event->topic_len) == 0) {
                if (strncmp(event->data, "move", event->data_len) == 0) {
                    ESP_LOGI(TAG, "Received move command, publishing response");
                    esp_mqtt_client_publish(event->client, "epuck/status", "Executing move command", 0, 1, 0);
                }
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
        /* Using the new SDK structure for the broker address */
        .broker = {
            .address.uri = "mqtt://10.42.0.1:1883",
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}




// Optionally include a JSON library if you want more flexibility
// #include "cJSON.h"

void publish_with_timestamp(const char* topic, const char* msg) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t timestamp = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
    
    char payload[256];
    snprintf(payload, sizeof(payload),
         "{\"seq\": %lu, \"msg\": \"%s\", \"timestamp\": %lld}",
         (unsigned long)msg_seq++, msg, timestamp);
    
    mqtt_publish(topic, payload);
}

