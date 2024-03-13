#include "time_count.h"
#include <time.h>

uint64_t xTaskGetTickCount(){
    struct timespec tms;
    
    /* The C11 way */
    if (! timespec_get(&tms, TIME_UTC)) { 

    /* POSIX.1-2008 way */
    //if (clock_gettime(CLOCK_REALTIME,&tms)) {
        return -1;
    }
    /* seconds, multiplied with 1 million */
    int64_t micros = tms.tv_sec * 1000000;
    /* Add full microseconds */
    micros += tms.tv_nsec/1000;
    /* round up if necessary */
    if (tms.tv_nsec % 1000 >= 500) {
        ++micros;
    }
    int64_t millisecond = micros / 1000;
    return millisecond;
}