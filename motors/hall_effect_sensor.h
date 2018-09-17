#ifndef HALL_EFFECT_SENSOR_H
#define HALL_EFFECT_SENSOR_H

#include <stdint.h>

int get_tick_count();
void hall_effect_init();
void sensor1_edge_change();
void sensor2_edge_change();

#endif // HALL_EFFECT_SENSOR_H
