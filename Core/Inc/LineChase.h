/*
 * LineChase.h
 *
 *  Created on: Nov 4, 2025
 *      Author: raito
 */

#ifndef INC_LINECHASE_H_
#define INC_LINECHASE_H_

#define SENSOR_ALL_DARK 20


#include "stm32f4xx_hal.h"
#include "Motor.h"
//#include "LineSensor.h"
#include "main.h"
#include "VelocityCtrl.h"
//#include "AngleCtrl.h"
#include <stdbool.h>      // 👈 bool型 (true/false) のために追加

#define LINE_SENSOR_NUM 16 // センサーの数を16個に定義

extern volatile int16_t Linesensor[LINE_SENSOR_NUM];

void calculateLineFollowingTermFlip(void);
void lineTraceFlip(void);

float getLineFollowingTerm(void);

void startLineTrace();
void stopLineTrace();

void checkCourseOut(void);
bool getCouseOutFlag(void);

void debugmotor(float, float);

#endif /* INC_LINECHASE_H_ */
