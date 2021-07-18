#pragma once

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include "stdio.h"
#endif

#ifdef ESP_PLATFORM
#define TAG "sigma_tcp"
#define LOG_INFO(...)   ESP_LOGI(TAG, __VA_ARGS__)
#define LOG_ERROR(...)  ESP_LOGE(TAG, __VA_ARGS__)
#else
#define LOG_INFO(...)   printf(__VA_ARGS__); printf("\n");
#define LOG_ERROR(...)  fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");
#endif //ESP_PLATFORM
