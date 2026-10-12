#ifndef INC_FLOW_SENSOR_H_
#define INC_FLOW_SENSOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void FlowSensor_Init(void);
float FlowSensor_ReadFlowRate_m3s(float dt);

// Dry-test flow simulation (Ozone-tunable, see flow_sensor.c):
// set flow_sim_enable=1 and write flow_sim_Lmin (signed, L/min).
extern volatile uint8_t flow_sim_enable;
extern volatile float   flow_sim_Lmin;

#ifdef __cplusplus
}
#endif

#endif /* INC_FLOW_SENSOR_H_ */
