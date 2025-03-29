
#ifndef __INCLUDE_MODBUS_H
#define __INCLUDE_MODBUS_H

#include <stdbool.h>
#include <modbus_defs.h>
#include <modbus_function.h>
#include <modbus_exception.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef void(* modbus_GenericFunctionCallback_t)(modbus_Pdu_t *, modbus_Pdu_t *);

typedef modbus_Exception_e(* modbus_ReadCallback_t)(modbus_FunctionCode_e, uint16_t, uint16_t *);
typedef modbus_Exception_e(* modbus_WriteCallback_t)(modbus_FunctionCode_e, uint16_t, uint16_t);

typedef struct
{
    uint8_t busAddress;
    modbus_Pdu_t pduRequest;
    modbus_Pdu_t pduResponse;

    modbus_GenericFunctionCallback_t pGenericFunctionHandler;
    modbus_ReadCallback_t pGenericReadHandler;
    modbus_WriteCallback_t pGenericWriteHandler;

    modbus_ReadCallback_t pReadCoilHandler;					// 0x01
    modbus_ReadCallback_t pReadDiscreteHandler;				// 0x02
    modbus_ReadCallback_t pReadHoldingRegisterHandler;		// 0x03, 0x17
    modbus_ReadCallback_t pReadInputRegisterHandler;		// 0x04
    modbus_ReadCallback_t pReadFifoQueueHandler;			// 0x18
    modbus_ReadCallback_t pReadExceptionStatusHandler;		// 0x07

    modbus_WriteCallback_t pWriteCoilHandler;				// 0x05, 0x0F
    modbus_WriteCallback_t pWriteRegisterHandler;			// 0x06, 0x10, 0x17

    /**
     * Special Handlers for:
     * - 0x08 Diagnostics
     * - 0x0B Get Comm Event Counter
     * - 0x0C Get Comm Event Log
     * - 0x11 Report Slave ID
     * - 0x14 Read File Record
     * - 0x15 Write File Record
     * - 0x16 Mask Write Register
     */
} modbus_t;



void modbus_ProcessData(modbus_t *pInstance);

void modbus_SetExceptionResponse(uint8_t exceptionCode, modbus_Pdu_t *pResponsePdu);

uint16_t modbus_EncodeAscii(char *pBuffer, uint16_t bufferSize, modbus_Pdu_t *pPdu);
bool modbus_DecodeAscii(const char *pData, uint16_t dataSize, modbus_Pdu_t *pPdu);

uint16_t modbus_EncodeRtu(uint8_t *pBuffer, uint16_t bufferSize, modbus_Pdu_t *pPdu);
bool modbus_DecodeRtu(const uint8_t *pData, uint16_t dataSize, modbus_Pdu_t *pPdu);

uint16_t modbus_GenerateCrc(modbus_Pdu_t *pPdu);
uint8_t modbus_GenerateLrc(modbus_Pdu_t *pPdu);



void modbus_AssertFailedHandler(const char *pFileName, uint32_t lineNumber);

#define MODBUS_ASSERT(x) if((x) == 0) modbus_AssertFailedHandler(__FILE__, __LINE__)



#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MODBUS_H */
