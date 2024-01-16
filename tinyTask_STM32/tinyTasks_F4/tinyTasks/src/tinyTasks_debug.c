
#include "tinyTasksError.h"
#include "tinyTasks_debug.h"

/*  This is most helpfull if I stick to having a single exit point
    for every function*/
const char* errorToString(TinyTasksStatus err) {
    switch (err) {
        case TINYTASKS_OK:                return "No error";
        case TINYTASKS_INVALID_TASK:      return "Invalid task";
        case TINYTASKS_INVALID_PRIORITY:  return "Invalid priority";
        case TINYTASKS_OUT_OF_MEMORY:     return "Out of memory";
        case TINYTASKS_TIMEOUT:           return "Timeout";
        case TINYTASKS_MAX_TASKS_REACHED: return "Max tasks reached";
        // Add more cases as needed
        default:                          return "Unknown Error";
    }
}