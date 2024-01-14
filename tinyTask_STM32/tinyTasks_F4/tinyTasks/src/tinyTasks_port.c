#include "tinyKernel.h"
#include "tinyTasksPort.h"
extern void printMsg(char *msg, ...);

void tinyTask_printMsg(char *msg, ...)
{	
    printMsg(msg);
}
