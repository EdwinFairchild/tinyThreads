#ifndef TINYTHREADSERROR_H
#define TINYTHREADSERROR_H
// clang-format off
 typedef enum TinyThreadsStatus {
    TINYTHREADS_OK = 0,           // No error
    TINYTHREADS_INVALID_TASK     = -1,     // Invalid thread
    TINYTHREADS_INVALID_PRIORITY = -2, // Invalid priority
    TINYTHREADS_OUT_OF_MEMORY    = -3,    // Out of memory
    TINYTHREADS_TIMEOUT = -4,          // Timeout
    TINYTHREADS_MAX_THREADS_REACHED = -5, // Max threads reached
   
    // Add more error codes as needed
}TinyThreadsStatus;

const char* errorToString(TinyThreadsStatus err);

#endif // TINYTHREADSERROR_H
