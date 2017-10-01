#pragma once

#include "config.h"

void init_globals();
void periph_setup();

bool dac_push(float ch1);
bool adc_pull(float &ch1, float &ch2);
