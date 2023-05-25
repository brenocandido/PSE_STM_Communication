#include "bufHandler.h"
#include <string.h>

#ifndef CUNIT_TEST
    #define UART_TRANSMIT(uart, data, bytes) HAL_UART_Transmit_IT(uart, data, bytes)
    #define UART_RECEIVE(uart, data, bytes) HAL_UART_Receive_DMA(uart, data, bytes)
#else
    #define UART_TRANSMIT(uart, data, bytes) 
    #define UART_RECEIVE(uart, data, bytes) 
#endif // CUNIT_TEST

static void _increaseBufIndex(int *pIndex, const int BUF_SIZE);

void bufHandler_init(BufHandler_t *pHandler, MsgBuffer_t *msgBuf, size_t bufSize)
{
    if (!pHandler || !msgBuf || bufSize <= 0)
    {
        return;
    }

    pHandler->msgBuf = msgBuf;
    memset((void *)pHandler->msgBuf, 0, sizeof(MsgBuffer_t) * bufSize);

    pHandler->bufRcvIndex = 0;
    pHandler->bufSendIndex = 0;

    pHandler->pRxUart = NULL;
    pHandler->pTxUart = NULL;

    pHandler->txAvailable = true;

    pHandler->BUF_SIZE = bufSize;
}

void bufHandler_setUart(BufHandler_t *pHandler, UART_HandleTypeDef *pTxUart, UART_HandleTypeDef *pRxUart)
{
    if (!pHandler || !pTxUart || !pRxUart)
    {
        return;
    }

    pHandler->pRxUart = pRxUart;
    pHandler->pTxUart = pTxUart;
}

bool bufHandler_checkEmpty(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return true;
    }

    return pHandler->bufSendIndex == pHandler->bufRcvIndex;
}

bool bufHandler_sendData(BufHandler_t *pHandler, MsgBuffer_t data)
{
    if (!pHandler)
    {
        return false;
    }

    if (!pHandler->txAvailable)
    {
        return false;
    }

    pHandler->txAvailable = false;

    UART_TRANSMIT(pHandler->pTxUart, (uint8_t *)data, MSG_TOTAL_BYTES);

    return true;
}

const uint8_t *bufHandler_getReceivedData(BufHandler_t *pHandler)
{
    if (!pHandler || !pHandler->msgBuf)
    {
        return NULL;
    }

    return pHandler->msgBuf[pHandler->bufRcvIndex];
}

void bufHandler_increaseRcvIndex(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return;
    }

    _increaseBufIndex(&pHandler->bufRcvIndex, pHandler->BUF_SIZE);
}

void bufHandler_increaseSendIndex(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return;
    }

    _increaseBufIndex(&pHandler->bufSendIndex, pHandler->BUF_SIZE);
}

bool bufHandler_transmitUartData(BufHandler_t *pHandler)
{
    if (!pHandler || !pHandler->pTxUart || !pHandler->msgBuf)
    {
        return false;
    }

    if (!pHandler->txAvailable)
    {
        return false;
    }

    HAL_StatusTypeDef ret = UART_TRANSMIT(pHandler->pTxUart, (uint8_t *)&pHandler->msgBuf[pHandler->bufSendIndex], MSG_TOTAL_BYTES);

    if (ret == HAL_OK)
    {
        pHandler->txAvailable = false;
        return true;
    }
    else
    {
        return false;
    }
}

void bufHandler_receiveUartData(BufHandler_t *pHandler)
{
    if (!pHandler || !pHandler->pRxUart || !pHandler->msgBuf)
    {
        return;
    }

	UART_RECEIVE(pHandler->pRxUart, (uint8_t *)&pHandler->msgBuf[pHandler->bufRcvIndex], MSG_TOTAL_BYTES);
}

void bufHandler_setTxAvailable(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return;
    }

    pHandler->txAvailable = true;
}

UART_HandleTypeDef *bufHandler_txUart(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return NULL;
    }

    return pHandler->pTxUart;
}

UART_HandleTypeDef *bufHandler_rxUart(BufHandler_t *pHandler)
{
    if (!pHandler)
    {
        return NULL;
    }

    return pHandler->pRxUart;
}

static void _increaseBufIndex(int *pIndex, const int BUF_SIZE)
{
	(*pIndex)++;
	if (*pIndex >= BUF_SIZE)
	{
		*pIndex = 0;
	}
}
