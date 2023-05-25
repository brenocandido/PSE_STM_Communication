#ifndef LEDTOGGLER_T
#define LEDTOGGLER_T

#include <stdint.h>
#include "stm32f407xx.h"

typedef struct _ledToggler
{
    uint8_t ledState;
    uint32_t cnt;
    uint32_t cntMax;
    GPIO_TypeDef *gpio;
    uint16_t ledPin;
} LedToggler_t;

void ledToggler_init(LedToggler_t *ledToggler, uint32_t maxCnt, GPIO_TypeDef *gpio, uint16_t ledPin);

void ledToggler_run(LedToggler_t *toggler);

#endif // LEDTOGGLER_T
