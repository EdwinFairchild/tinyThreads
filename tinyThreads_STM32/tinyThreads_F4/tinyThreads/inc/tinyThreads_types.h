#ifndef TINYTHREADS_TYPES_H
#define TINYTHREADS_TYPES_H

#include "stdbool.h"
#include "stdint.h"
#include "tinyThreads_error.h"

typedef uint32_t tinyThreadsTime_ms_t;
typedef uint32_t tinyThreadPeriod_t;
typedef uint32_t tinyThreadPriority_t;
typedef int32_t  tinyThread_tcb_idx;
typedef uint32_t tinyThread_state;

#define THREAD_STATE_READY          (0x00)
#define THREAD_STATE_BLOCKED        (1 << 1)
#define THREAD_STATE_SLEEPING       (1 << 2)
#define THREAD_STATE_PAUSED         (1 << 3)
#define THREAD_STATE_PENDING_NOTIFY (1 << 4)
/* Thread Control Block */
typedef struct tinyThread_tcb tinyThread_tcb_t;

typedef struct tinyThread_tcb
{
    /*!!!!!!! Dont change order of this without updating core.h offsets */
    uint32_t            *stackPointer;         // Pointer to the current stack pointer
    tinyThread_tcb_t    *next;                 // Pointer to the next thread
    tinyThread_tcb_t    *prev;                 // Pointer to the previous thread
    tinyThreadPeriod_t   period_ms;            // Period of the thread
    tinyThreadsTime_ms_t lastRunTime;          // Last time the thread ran
    tinyThreadPriority_t priority;             // Priority of the thread
    tinyThread_state     state;                // State of the thread
    uint32_t            *notifyVal;            // Value to be passed during notification
    tinyThreadsTime_ms_t sleep_count_ms;       // sleep count in ms
    tinyThreadsTime_ms_t notify_timeout_count; // timeout counter
    tinyThread_tcb_idx   id;                   // Unique thread identifier
    uint8_t              name[16];             // Name of the thread
} tinyThread_tcb;

/* Linkedlist for ready threads */
typedef struct tinyThread_ready_threads_node
{
    tinyThread_tcb                       *tcb;
    struct tinyThread_ready_threads_node *next;
    struct tinyThread_ready_threads_node *prev;
} tinyThread_ready_threads_node_t;

typedef struct
{
    tinyThread_tcb *tcb_ll_head;
    tinyThread_tcb *tcb_ll_tail;

} tinyThread_ready_threads_list_t;

/* Linkedlist for suspended threads */
typedef struct tinyThread_suspended_threads_node
{
    tinyThread_tcb                           *tcb;
    struct tinyThread_suspended_threads_node *next;
    struct tinyThread_suspended_threads_node *prev;
} tinyThread_suspended_threads_node_t;

typedef struct
{
    tinyThread_suspended_threads_node_t *head;
    tinyThread_suspended_threads_node_t *tail;

} tinyThread_suspended_threads_list_t;

#endif // TINYTHREADS_TYPES_H
