#ifndef MQTT_H
#define MQTT_H
#include "mqtt_client.h"


// Starts the MQTT client and connects to the broker.
void mqtt_app_start(void);
void mqtt_publish(const char *topic, const char *data);
void publish_with_timestamp(const char* topic, const char* msg);
extern uint32_t messages_published;
extern uint32_t bytes_published;
extern uint32_t publish_errors;
extern SemaphoreHandle_t mqtt_connected_sem;


#endif // MQTT_H
