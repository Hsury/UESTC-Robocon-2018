#ifndef __PROBE_H
#define __PROBE_H

#include "Includes.h"

void Probe_ReportOnline(void);
void Probe_SetUser(float Data);
void Probe_SetTimer(uint8_t Index, uint32_t Timer);
void Probe_SetArrive(uint8_t TZx);

#endif
