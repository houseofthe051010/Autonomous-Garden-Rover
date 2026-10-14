#include "system_metrics.h"
#include "driver/temperature_sensor.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static float temperature, cpu[2];
static bool temp_valid, cpu_valid;
static temperature_sensor_handle_t sensor;
static void sample(void *arg)
{
    (void)arg;
    static TaskStatus_t tasks[64];
    uint32_t previous[2] = {0}, previous_time = 0;
    bool seeded = false;
    for (;;) {
        configRUN_TIME_COUNTER_TYPE total = 0;
        UBaseType_t n = uxTaskGetSystemState(tasks, 64, &total);
        uint32_t idle[2] = {0};
        bool found[2] = {false};
        for (unsigned i=0;i<n;i++) {
            for (int c=0;c<2;c++) {
                const char *name = c ? "IDLE1" : "IDLE0";
                if (!strcmp(tasks[i].pcTaskName,name)) {
                    idle[c] = tasks[i].ulRunTimeCounter;
                    found[c] = true;
                }
            }
        }
        uint32_t elapsed = (uint32_t)total - previous_time;
        float load[2] = {0};
        bool valid = seeded && found[0] && found[1] && elapsed;
        if (valid) for (int c=0;c<2;c++)
            load[c] = fmaxf(0, fminf(100, 100.0f*(1.0f-(float)(idle[c]-previous[c])/elapsed)));
        float t = 0;
        bool tv = sensor && temperature_sensor_get_celsius(sensor,&t)==ESP_OK && isfinite(t);
        portENTER_CRITICAL(&lock);
        temperature=t; temp_valid=tv; cpu_valid=valid;
        memcpy(cpu,load,sizeof(cpu));
        portEXIT_CRITICAL(&lock);
        seeded=found[0]&&found[1]; previous_time=total;
        memcpy(previous,idle,sizeof(idle));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void system_metrics_start(void)
{
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    if (temperature_sensor_install(&config,&sensor)==ESP_OK) {
        if (temperature_sensor_enable(sensor)!=ESP_OK) {
            temperature_sensor_uninstall(sensor); sensor=NULL;
        }
    }
    /* Optional diagnostics must not prevent camera boot if allocation fails. */
    xTaskCreate(sample,"system_metrics",3072,NULL,1,NULL);
}
void system_metrics_json(char *buffer,unsigned size)
{
    char t[24]="null", a[24]="null", b[24]="null";
    portENTER_CRITICAL(&lock);
    float temp=temperature, c0=cpu[0], c1=cpu[1];
    bool tv=temp_valid, cv=cpu_valid;
    portEXIT_CRITICAL(&lock);
    if(tv)snprintf(t,sizeof(t),"%.1f",temp);
    if(cv){snprintf(a,sizeof(a),"%.1f",c0);snprintf(b,sizeof(b),"%.1f",c1);}
    size_t total=heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t free=heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    snprintf(buffer,size,"{\"temperature_c\":%s,\"cpu_percent\":[%s,%s],\"psram_total\":%u,\"psram_used\":%u}",
        t,a,b,(unsigned)total,(unsigned)(total>free?total-free:0));
}
