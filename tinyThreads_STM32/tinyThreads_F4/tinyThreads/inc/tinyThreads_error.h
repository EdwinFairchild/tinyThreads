#ifndef TINYTASKSERROR_H
#define TINYTASKSERROR_H

 typedef enum TinyThreadsStatus {
    TINYTASKS_OK = 0,           // No error
    TINYTASKS_INVALID_TASK     = -1,     // Invalid thread
    TINYTASKS_INVALID_PRIORITY = -2, // Invalid priority
    TINYTASKS_OUT_OF_MEMORY    = -3,    // Out of memory
    TINYTASKS_TIMEOUT = -4,          // Timeout
    TINYTASKS_MAX_TASKS_REACHED = -5, // Max threads reached
   
    // Add more error codes as needed
}TinyThreadsStatus;

const char* errorToString(TinyThreadsStatus err);

#endif // TINYTASKSERROR_H
