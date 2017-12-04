/*
 * PID.c
 *
 *  Created on: 02/12/2017
 *      Author: Roberto
 */

#include "PID.h"
#include "DataTypeDefinitions.h"
#include "ACELEROMETRO.h"
#include "GlobalFunctions.h"
#include <stdio.h>

uint8 prevError = 0;
uint8 integral = 0;
uint8 derivative = 0;
sint8 error = 0;
uint16 Kp = 1;
uint16 Ki = 0;
uint16 Kd = 500;
uint8 iteration_time = 0.5;
const uint8 inputRef = 125;
uint32 output;

uint32 PID(void){
	error = inputRef - getValue_Y();
	printf("\n y: %d", getValue_Y());
	printf("\n error: %d", error);
	integral = integral + (error * iteration_time);
	derivative = (error- prevError) / iteration_time;
	output = Kp * error + Ki * integral + Kd * derivative;
	printf("\n output: %d", output);
	prevError = error;
	delay(65000);
	return output;
}
