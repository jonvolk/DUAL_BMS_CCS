#if !defined(__MY_MATH_H_)
#define __MY_MATH_H_

#include <stdio.h>

#define MAP(x, in_min, in_max, out_min,out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#define RAMPUP(current, target, rate) ((target < current || (current + rate) > target) ? target : current + rate)
#define RAMPDOWN(current, target, rate) ((target > current || (current - rate) < target) ? target : current - rate)
//#define DUTY(pcnt)((65535/100)*pcnt)

//Fixed Point format 




#endif // __MY_MATH_H_






