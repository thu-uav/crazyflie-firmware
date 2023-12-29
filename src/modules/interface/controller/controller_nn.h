
#ifndef __CONTROLLER_NN_H__
#define __CONTROLLER_NN_H__

#include "stabilizer_types.h"
// #include "network_evaluate.h"

#include <math.h>

/*
 * since the network outputs thrust on each motor,
 * we need to define a struct which stores the values
*/
typedef struct control_t_n {
	float thrust_0; 
	float thrust_1;
	float thrust_2;
	float thrust_3;	
} control_t_n;

void networkEvaluate(control_t_n *control_n, const float *state_array);

void controllerNNInit(void);
bool controllerNNTest(void);
void controllerNN(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

void thrusts2PWM(control_t_n *control_n, 
	int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3);

float scale(float v);

float clip(float v, float min, float max);

void controllerNNEnableBigQuad(void);

#endif //__CONTROLLER_NN_H__

