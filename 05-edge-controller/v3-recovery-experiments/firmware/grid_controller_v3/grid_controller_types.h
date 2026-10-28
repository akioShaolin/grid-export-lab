#pragma once

#include <Arduino.h>

enum MeterCycleStage : uint8_t {
  METER_IDLE = 0,
  METER_WAIT_STATUS_A,
  METER_WAIT_RATIOS,
  METER_WAIT_POWER,
  METER_WAIT_STATUS_B
};

struct StatsBucket {
  uint32_t seq;
  uint32_t bootCount;
  uint32_t endUptimeS;
  uint32_t durationS;
  uint32_t samples;

  float meterAvgW;
  float meterMinW;
  float meterMaxW;

  float invAvgW;
  float invMinW;
  float invMaxW;

  float setpointAvgW;
  float errorAvgW;
  float commandAvgW;

  uint8_t onlineMask;
  uint16_t alarmFlags;
};