#ifndef BUFHANDLER_H_
#define BUFHANDLER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef CUNIT_TEST
    #include "stm32f4xx_hal.h"
#else
    #define HAL_OK  true

    typedef void *UART_HandleTypeDef;
    typedef bool HAL_StatusTypeDef;
#endif // CUNIT_TEST


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

/// \brief Initializes the BufHandler.
bool bufHandler_init(BufHandler_t *pHandler, MsgBuffer_t *msgBuf, size_t bufSize);


/// \brief Configures the UART handlers for the BufHandler.
///
/// The TX and RX UART are configure separately because they don't necessarily
/// receive from and transmit to the same interface.
bool bufHandler_setUart(BufHandler_t *pHandler, UART_HandleTypeDef *pTxUart, UART_HandleTypeDef *pRxUart);

/// \brief Checks whether there's anything to be sent in the BufHandler.
///
/// A BufHandler is not empty when it has received anything through its RX uart,
/// meaning it must now forward it to the TX.
///
/// \return true if the BufHandler is empty, else false.
bool bufHandler_checkEmpty(BufHandler_t *pHandler);

/// \brief Forces a send through the BufHandler TX UART.
///
/// This function bypasses the buffer itself to force a send through the TX
/// UART. Although it will only successfully send if the UART is available at
/// the moment.
///
/// \param data Data to be sent. 
/// \return true if the send was successful, else false.
bool bufHandler_sendData(BufHandler_t *pHandler, MsgBuffer_t data);

/// \brief Returns the last received MsgBuffer.
const uint8_t *bufHandler_getReceivedData(BufHandler_t *pHandler);

/// \brief Increases the receive index for the message buffer.
///
/// A receive index may be increased after it has successfully received a
/// message.
bool bufHandler_increaseRcvIndex(BufHandler_t *pHandler);

/// \brief Increases the send index for the message buffer.
///
/// A send index may be increased after the data has been successfully
/// transmitted or if it must be discarded.
bool bufHandler_increaseSendIndex(BufHandler_t *pHandler);

/// \brief Transmits the data currently pointed by the send index.
///
/// This function will only successfully transmit the data if the TX is
/// available at the moment of calling. In a successful transmit, the TX
/// available flag will be set to false again.
///
/// \return true for a successful transmission, else false.
bool bufHandler_transmitUartData(BufHandler_t *pHandler);

/// \brief Sets data to be received at the current receive index.
void bufHandler_receiveUartData(BufHandler_t *pHandler);

/// \brief Sets the TX UART  available flag to true.
bool bufHandler_setTxAvailable(BufHandler_t *pHandler);

/// \brief Returns the pointer to the configured TX UART.
UART_HandleTypeDef *bufHandler_txUart(BufHandler_t *pHandler);

/// \brief Returns the pointer to the configured RX UART.
UART_HandleTypeDef *bufHandler_rxUart(BufHandler_t *pHandler);

#endif // BUFHANDLER_H_
