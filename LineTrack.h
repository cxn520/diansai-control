#ifndef __LineTrack_h_
#define __LineTrack_h_
#include "main.h"
extern uint8_t baixian_FLAG;
extern uint8_t error;
void LineTrack();
int8_t LinePID_Error();
int16_t LinePID_Calc(int8_t err);

extern int hw1,hw2,hw3,hw4,hw5,hw6,hw7,hw8;

#endif