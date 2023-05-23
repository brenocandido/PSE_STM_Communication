#include "ledToggler.h"

void ledToggler_init(LedToggler_t *ledToggler, uint32_t maxCnt, GPIO_TypeDef *gpio, uint16_t ledPin)
{
    ledToggler->ledState = 0;
    ledToggler->cnt = 0;
    ledToggler->cntMax = maxCnt;
    ledToggler->gpio = gpio;
    ledToggler->ledPin = ledPin;
}

void ledToggler_run(LedToggler_t *toggler)
{
    const uint32_t LED_ON = toggler->ledPin << 16;
	const uint32_t LED_OFF = toggler->ledPin;

    toggler->cnt++;
    if (toggler->cnt >= toggler->cntMax)
    {
        toggler->cnt = 0;
        toggler->ledState = !toggler->ledState;

        if (toggler->ledState)
        {
            toggler->gpio->BSRR = LED_ON;
        }
        else
        {
            toggler->gpio->BSRR = LED_OFF;
        }
    }
}