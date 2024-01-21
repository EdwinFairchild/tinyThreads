
#include "tinyThreads_error.h"
#include "tinyThreads_debug.h"

/*  This is most helpfull if I stick to having a single exit point
    for every function*/
const char* errorToString(TinyThreadsStatus err) {
    switch (err) {
        case TINYTHREADS_OK:                return "No error";
        case TINYTHREADS_INVALID_TASK:      return "Invalid thread";
        case TINYTHREADS_INVALID_PRIORITY:  return "Invalid priority";
        case TINYTHREADS_OUT_OF_MEMORY:     return "Out of memory";
        case TINYTHREADS_TIMEOUT:           return "Timeout";
        case TINYTHREADS_MAX_TASKS_REACHED: return "Max threads reached";
        // Add more cases as needed
        default:                          return "Unknown Error";
    }
}