      
#include "debug.h"

#include "stdio.h"
#include "stdint.h"

#define PRINT(title, fmt, args...) printf("{"#title"}"fmt"\n", ##args)


void debug_send(float data1, float data2, float data3,float data4, float data5, float data6)
{
	PRINT(plotter, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f", data1, data2, data3, data4, data5, data6);
}

