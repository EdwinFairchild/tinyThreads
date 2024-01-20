
#include "tinyThreads_error.h"
#include "tinyThreads_debug.h"

/*  This is most helpfull if I stick to having a single exit point
    for every function*/
const char* errorToString(TinyThreadsStatus err) {
    switch (err) {
        case TINYTASKS_OK:                return "No error";
        case TINYTASKS_INVALID_TASK:      return "Invalid thread";
        case TINYTASKS_INVALID_PRIORITY:  return "Invalid priority";
        case TINYTASKS_OUT_OF_MEMORY:     return "Out of memory";
        case TINYTASKS_TIMEOUT:           return "Timeout";
        case TINYTASKS_MAX_TASKS_REACHED: return "Max threads reached";
        // Add more cases as needed
        default:                          return "Unknown Error";
    }
}