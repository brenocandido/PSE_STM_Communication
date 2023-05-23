#ifndef BUFHANDLER_H_
#define BUFHANDLER_H_

#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define MSG_TOTAL_BYTES			13

typedef uint8_t MsgBuffer_t[MSG_TOTAL_BYTES];

typedef struct _BufHandler
{
    MsgBuffer_t *msgBuf;

    int bufRcvIndex;
    int bufSendIndex;

    UART_HandleTypeDef *pTxUart;
    UART_HandleTypeDef *pRxUart;

    bool txAvailable;

    size_t BUF_SIZE;

} BufHandler_t;

void bufHandler_init(BufHandler_t *pHandler, MsgBuffer_t *msgBuf, size_t bufSize);

void bufHandler_setUart(BufHandler_t *pHandler, UART_HandleTypeDef *pTxUart, UART_HandleTypeDef *pRxUart);

bool bufHandler_checkEmpty(BufHandler_t *pHandler);

bool bufHandler_sendData(BufHandler_t *pHandler, MsgBuffer_t data);

void bufHandler_increaseRcvIndex(BufHandler_t *pHandler);

void bufHandler_increaseSendIndex(BufHandler_t *pHandler);

bool bufHandler_transmitUartData(BufHandler_t *pHandler);

void bufHandler_receiveUartData(BufHandler_t *pHandler);

void bufHandler_setTxAvailable(BufHandler_t *pHandler);

UART_HandleTypeDef *bufHandler_txUart(BufHandler_t *pHandler);

#endif // BUFHANDLER_H_
