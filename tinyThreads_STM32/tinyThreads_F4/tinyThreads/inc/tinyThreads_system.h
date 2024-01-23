#ifndef TINYTASK_SYSTEM_H
#define TINYTASK_SYSTEM_H
// clang-format off
#include "stdint.h"
/*************************************************
 * Critical sections should be nested
 * and only the outermost critical section
 * should enable/disable interrupts
 *************************************************/
void tinyThreads_sys_CsEnter(void);
void tinyThreads_sys_CsExit(void);
#endif // TINYTASK_SYSTEM_H
