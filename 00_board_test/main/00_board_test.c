#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    printf("\n");
    printf("====================================\n");
    printf("ESP32-S3 Board Test Start!\n");
    printf("====================================\n");
    int count = 0;
    while (1)
    {
        printf("Hello ESP32-S3! count = %d\n", count);
        count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
