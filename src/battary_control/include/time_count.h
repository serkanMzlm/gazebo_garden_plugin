#ifndef _TIME_COUNT_
#define _TIME_COUNT_

#include <stdint.h>

/**
 * Anlık zamanı hesaplar.
 * @return uint64_t milliseconds
*/
uint64_t xTaskGetTickCount(void);

#endif