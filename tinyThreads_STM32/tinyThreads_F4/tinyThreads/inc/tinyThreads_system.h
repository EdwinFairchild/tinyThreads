#ifndef TINYTASK_SYSTEM_H
#define TINYTASK_SYSTEM_H

#include "stdint.h"
#include "tinyThreads_port.h"
/*************************************************
 * Critical sections should be nested
 * and only the outermost critical section
 * should enable/disable interrupts
 *************************************************/
void tinyThreads_sys_CsEnter(void);
void tinyThreads_sys_CsExit(void);
#endif // TINYTASK_SYSTEM_H
