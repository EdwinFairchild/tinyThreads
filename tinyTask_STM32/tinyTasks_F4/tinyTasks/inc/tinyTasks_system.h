#ifndef TINYTASK_SYSTEM_H
#define TINYTASK_SYSTEM_H

#include "stdint.h"
/*************************************************
* Critical sections should be nested 
* and only the outermost critical section
* should enable/disable interrupts
*************************************************/
void tinyTasks_sys_CsEnter(void);
void tinyTasks_sys_CsExit(void);
#endif // TINYTASK_SYSTEM_H
