#pragma once

#include <stdbool.h>
#include <stdint.h>

bool drive_control_autonomous_begin(void);
bool drive_control_autonomous_set_percent(int left_percent, int right_percent);
bool drive_control_autonomous_get_encoders(uint16_t adc[4], uint32_t *sequence);
void drive_control_autonomous_end(void);
