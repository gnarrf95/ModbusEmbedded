
#include <stdio.h>
#include <stddef.h>

#include <modbus.h>
#include <modbus_function.h>
#include <modbus_exception.h>



static void modbus_ProcessRead(modbus_t *pInstance);
static void modbus_ProcessReadBit(modbus_Pdu_t *pResponsePdu, modbus_ReadCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity);
static void modbus_ProcessReadRegister(modbus_Pdu_t *pResponsePdu, modbus_ReadCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity);

static void modbus_ProcessWriteSingle(modbus_t *pInstance);

static void modbus_ProcessWriteMultiple(modbus_t *pInstance);
static void modbus_ProcessWriteMultipleBits(modbus_Pdu_t *pResponsePdu, modbus_WriteCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity, uint8_t *pByteBuffer);
static void modbus_ProcessWriteMultipleRegisters(modbus_Pdu_t *pResponsePdu, modbus_WriteCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity, uint8_t *pByteBuffer);

//------------------------------------------------------------------------------
//
void modbus_ProcessData(modbus_t *pInstance)
{
	MODBUS_ASSERT(pInstance != NULL);
	MODBUS_ASSERT(pInstance->pGenericFunctionHandler != NULL);

	pInstance->pduResponse.functionCode = pInstance->pduRequest.functionCode;
	pInstance->pduResponse.busAddress = pInstance->pduRequest.busAddress;

    switch(pInstance->pduRequest.functionCode)
    {
        // Read Functions
        case MODBUS_FUNCTION_READCOILS:
        case MODBUS_FUNCTION_READDISCRETE:
        case MODBUS_FUNCTION_READHOLDING:
        case MODBUS_FUNCTION_READINPUT:
        {
            modbus_ProcessRead(pInstance);
            break;
        }

//        case MODBUS_FUNCTION_READFIFO:
//        case MODBUS_FUNCTION_READEXCEPTION:

        // Write-Single Functions
        case MODBUS_FUNCTION_WRITESINGLE_COIL:
        case MODBUS_FUNCTION_WRITESINGLE_REG:
        {
        	modbus_ProcessWriteSingle(pInstance);
            break;
        }

        // Write-Multiple Functions
        case MODBUS_FUNCTION_WRITEMULT_COILS:
        case MODBUS_FUNCTION_WRITEMULT_REGS:
        {
        	modbus_ProcessWriteMultiple(pInstance);
            break;
        }

        default:
        {
            // Illegal Function Exception
            printf("Function [%02u] not implemented yet.\n", pInstance->pduRequest.functionCode);
            modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALFUNCTION, &pInstance->pduResponse);
            break;
        }
    }
}

//------------------------------------------------------------------------------
//
void modbus_SetExceptionResponse(modbus_Exception_e exceptionCode, modbus_Pdu_t *pResponsePdu)
{
	pResponsePdu->functionCode |= 0x80;
	pResponsePdu->pPayload[0] = (uint8_t)exceptionCode;
	pResponsePdu->payloadSize = 1;
}



//------------------------------------------------------------------------------
//
__attribute__((weak)) void modbus_AssertFailedHandler(const char *pFileName, uint32_t lineNumber)
{
	printf("MODBUS_ASSERT() failed at [ %s : %lu ].\r\n", pFileName, lineNumber);
    while(1)
    {}
}



//------------------------------------------------------------------------------
//
static void modbus_ProcessRead(modbus_t *pInstance)
{
	MODBUS_ASSERT(pInstance != NULL);
	MODBUS_ASSERT(pInstance->pGenericFunctionHandler != NULL);

	if(pInstance->pduRequest.payloadSize != 4)
	{
		modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
		return;
	}

    modbus_ReadCallback_t pCallback = NULL;
    switch(pInstance->pduRequest.functionCode)
    {
        case MODBUS_FUNCTION_READCOILS:
        {
        	pCallback = pInstance->pReadCoilHandler;
            break;
        }

        case MODBUS_FUNCTION_READDISCRETE:
        {
        	pCallback = pInstance->pReadDiscreteHandler;
            break;
        }

        case MODBUS_FUNCTION_READHOLDING:
        {
        	pCallback = pInstance->pReadHoldingRegisterHandler;
            break;
        }

        case MODBUS_FUNCTION_READINPUT:
        {
        	pCallback = pInstance->pReadInputRegisterHandler;
            break;
        }

        default:
        {
        	modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALFUNCTION, &pInstance->pduResponse);
        	return;
        }
    }

    if(pCallback == NULL)
    {
        if(pInstance->pGenericReadHandler == NULL)
        {
        	pInstance->pGenericFunctionHandler(&pInstance->pduRequest, &pInstance->pduResponse);
        	return;
        }
        else
        {
        	pCallback = pInstance->pGenericReadHandler;
        }
    }



    uint16_t startAddress = 0;
    uint16_t quantity = 0;

    startAddress |= (uint16_t)pInstance->pduRequest.pPayload[0] << 8;
    startAddress |= (uint16_t)pInstance->pduRequest.pPayload[1];

    quantity |= (uint16_t)pInstance->pduRequest.pPayload[2] << 8;
    quantity |= (uint16_t)pInstance->pduRequest.pPayload[3];

    if(pInstance->pduRequest.functionCode == MODBUS_FUNCTION_READCOILS ||
    	pInstance->pduRequest.functionCode == MODBUS_FUNCTION_READDISCRETE)
    {
        if(quantity < 0x0001 || quantity > MODBUS_READ_BIT_MAX_QUANTITY)
        {
            // Illegal data value
            modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
        }
        else
        {
            modbus_ProcessReadBit(&pInstance->pduResponse, pCallback, pInstance->pduRequest.functionCode, startAddress, quantity);
        }
    }
    else
    {
        if(quantity < 0x0001 || quantity > MODBUS_READ_REGISTER_MAX_QUANTITY)
        {
            // Illegal data value
            modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
        }
        else
        {
            modbus_ProcessReadRegister(&pInstance->pduResponse, pCallback, pInstance->pduRequest.functionCode, startAddress, quantity);
        }
    }
}

//------------------------------------------------------------------------------
//
static void modbus_ProcessReadBit(modbus_Pdu_t *pResponsePdu, modbus_ReadCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity)
{
    MODBUS_ASSERT(pResponsePdu != NULL);
    MODBUS_ASSERT(pCallback != NULL);

	uint16_t valueBuffer = 0;
	modbus_Exception_e ret = 0;
    uint16_t bitCtr = 0;

    pResponsePdu->pPayload[0] = 0;
	pResponsePdu->payloadSize = 1;
    pResponsePdu->pPayload[1] = 0;

    while(((pResponsePdu->payloadSize - 1) * 8 + bitCtr) < quantity)
    {
        const uint16_t address = (pResponsePdu->payloadSize - 1) * 8 + bitCtr;

        ret = pCallback(functionCode, address, &valueBuffer);
        if(ret != MODBUS_EXCEPTION_SUCCESS)
        {
            modbus_SetExceptionResponse(ret, pResponsePdu);
            return;
        }

        if(valueBuffer == MODBUS_BIT_ON)
        {
            pResponsePdu->pPayload[pResponsePdu->payloadSize] |= (1 << bitCtr);
        }

        bitCtr++;
        if(bitCtr >= 8)
        {
            bitCtr = 0;
            pResponsePdu->payloadSize++;
            pResponsePdu->pPayload[pResponsePdu->payloadSize] = 0;
        }
    }

    pResponsePdu->pPayload[0] = (pResponsePdu->payloadSize - 1);
}

//------------------------------------------------------------------------------
//
static void modbus_ProcessReadRegister(modbus_Pdu_t *pResponsePdu, modbus_ReadCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity)
{
    MODBUS_ASSERT(pResponsePdu != NULL);
    MODBUS_ASSERT(pCallback != NULL);

	uint16_t valueBuffer = 0;
	modbus_Exception_e ret = 0;

	pResponsePdu->pPayload[0] = (uint8_t)(2 * quantity);
	pResponsePdu->payloadSize = 1;

    for(uint16_t ctr = 0; ctr < quantity; ctr++)
    {
        ret = pCallback(functionCode, startAddress + ctr, &valueBuffer);
        if(ret != MODBUS_EXCEPTION_SUCCESS)
        {
            modbus_SetExceptionResponse(ret, pResponsePdu);
            return;
        }

        pResponsePdu->pPayload[1 + (ctr * 2)] = (uint8_t)((valueBuffer >> 8) & 0xFF);
        pResponsePdu->pPayload[1 + (ctr * 2 + 1)] = (uint8_t)(valueBuffer & 0xFF);
        pResponsePdu->payloadSize += 2;
    }
}


//------------------------------------------------------------------------------
//
static void modbus_ProcessWriteSingle(modbus_t *pInstance)
{
	MODBUS_ASSERT(pInstance != NULL);
	MODBUS_ASSERT(pInstance->pGenericFunctionHandler != NULL);

	if(pInstance->pduRequest.payloadSize != 4)
	{
		modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
		return;
	}

	modbus_WriteCallback_t pCallback = NULL;
	switch(pInstance->pduRequest.functionCode)
	{
		case MODBUS_FUNCTION_WRITESINGLE_COIL:
		{
			pCallback = pInstance->pWriteCoilHandler;
			break;
		}

		case MODBUS_FUNCTION_WRITESINGLE_REG:
		{
			pCallback = pInstance->pWriteRegisterHandler;
			break;
		}

		default:
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALFUNCTION, &pInstance->pduResponse);
			return;
		}
	}

	if(pCallback == NULL)
	{
		if(pInstance->pGenericWriteHandler == NULL)
		{
			pInstance->pGenericFunctionHandler(&pInstance->pduRequest, &pInstance->pduResponse);
			return;
		}
		else
		{
			pCallback = pInstance->pGenericWriteHandler;
		}
	}



	uint16_t address = 0;
	uint16_t value = 0;

	address |= (uint16_t)pInstance->pduRequest.pPayload[0] << 8;
	address |= (uint16_t)pInstance->pduRequest.pPayload[1];

	value |= (uint16_t)pInstance->pduRequest.pPayload[2] << 8;
	value |= (uint16_t)pInstance->pduRequest.pPayload[3];

	if(pInstance->pduRequest.functionCode == MODBUS_FUNCTION_WRITESINGLE_COIL)
	{
		if((value != MODBUS_BIT_ON) && (value != MODBUS_BIT_OFF))
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
			return;
		}
	}

	modbus_Exception_e ret = pCallback(pInstance->pduRequest.functionCode, address, value);
	if(ret != MODBUS_EXCEPTION_SUCCESS)
	{
		modbus_SetExceptionResponse(ret, &pInstance->pduResponse);
	}

	pInstance->pduResponse.functionCode = pInstance->pduRequest.functionCode;
	pInstance->pduResponse.payloadSize = 4;
	pInstance->pduResponse.pPayload[0] = (address >> 8) & 0xFF;
	pInstance->pduResponse.pPayload[1] = address & 0xFF;
	pInstance->pduResponse.pPayload[2] = (value >> 8) & 0xFF;
	pInstance->pduResponse.pPayload[3] = value & 0xFF;
}

//------------------------------------------------------------------------------
//
static void modbus_ProcessWriteMultiple(modbus_t *pInstance)
{
	MODBUS_ASSERT(pInstance != NULL);
	MODBUS_ASSERT(pInstance->pGenericFunctionHandler != NULL);

	if(pInstance->pduRequest.payloadSize < 5)
	{
		modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
		return;
	}

	modbus_WriteCallback_t pCallback = NULL;
	switch(pInstance->pduRequest.functionCode)
	{
		case MODBUS_FUNCTION_WRITEMULT_COILS:
		{
			pCallback = pInstance->pWriteCoilHandler;
			break;
		}

		case MODBUS_FUNCTION_WRITEMULT_REGS:
		{
			pCallback = pInstance->pWriteRegisterHandler;
			break;
		}

		default:
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALFUNCTION, &pInstance->pduResponse);
			return;
		}
	}

	if(pCallback == NULL)
	{
		if(pInstance->pGenericWriteHandler == NULL)
		{
			pInstance->pGenericFunctionHandler(&pInstance->pduRequest, &pInstance->pduResponse);
			return;
		}
		else
		{
			pCallback = pInstance->pGenericWriteHandler;
		}
	}



	uint16_t startAddress = 0;
	uint16_t quantity = 0;
	uint8_t byteCount = 0;

	startAddress |= (uint16_t)pInstance->pduRequest.pPayload[0] << 8;
	startAddress |= (uint16_t)pInstance->pduRequest.pPayload[1];

	quantity |= (uint16_t)pInstance->pduRequest.pPayload[2] << 8;
	quantity |= (uint16_t)pInstance->pduRequest.pPayload[3];

	byteCount = pInstance->pduRequest.pPayload[4];

	if(pInstance->pduRequest.functionCode == MODBUS_FUNCTION_WRITEMULT_COILS)
	{
		if(
			(quantity > (byteCount * 8)) ||
			(quantity < (byteCount * 8 - 1)))
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
			return;
		}
		else if(quantity < 1 || quantity > MODBUS_WRITE_BIT_MAX_QUANTITY)
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
			return;
		}
		else
		{
			modbus_ProcessWriteMultipleBits(&pInstance->pduResponse, pCallback, pInstance->pduRequest.functionCode, startAddress, quantity, &pInstance->pduRequest.pPayload[5]);
		}
	}
	else
	{
		if(quantity != (byteCount / 2))
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
			return;
		}
		else if(quantity < 1 || quantity > MODBUS_WRITE_REGISTER_MAX_QUANTITY)
		{
			modbus_SetExceptionResponse(MODBUS_EXCEPTION_ILLEGALDATAVALUE, &pInstance->pduResponse);
			return;
		}
		else
		{
			modbus_ProcessWriteMultipleRegisters(&pInstance->pduResponse, pCallback, pInstance->pduRequest.functionCode, startAddress, quantity, &pInstance->pduRequest.pPayload[5]);
		}
	}
}

//------------------------------------------------------------------------------
//
static void modbus_ProcessWriteMultipleBits(modbus_Pdu_t *pResponsePdu, modbus_WriteCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity, uint8_t *pByteBuffer)
{
	MODBUS_ASSERT(pResponsePdu != NULL);
	MODBUS_ASSERT(pCallback != NULL);
	MODBUS_ASSERT(pByteBuffer != NULL);

	uint16_t valueBuffer = 0;
	modbus_Exception_e ret = 0;
	uint16_t bufferCtr = 0;
	uint16_t bitCtr = 0;

	while((bufferCtr * 8 + bitCtr) < quantity)
	{
		const uint16_t address = startAddress + (bufferCtr * 8 + bitCtr);
		valueBuffer = (pByteBuffer[bufferCtr] & (1 << bitCtr)) ? MODBUS_BIT_ON : MODBUS_BIT_OFF;

		ret = pCallback(functionCode, address, valueBuffer);
		if(ret != MODBUS_EXCEPTION_SUCCESS)
		{
			modbus_SetExceptionResponse(ret, pResponsePdu);
			return;
		}

		bitCtr++;
		if(bitCtr >= 8)
		{
			bitCtr = 0;
			bufferCtr++;
		}
	}

	pResponsePdu->functionCode = functionCode;
	pResponsePdu->payloadSize = 4;
	pResponsePdu->pPayload[0] = (startAddress >> 8) & 0xFF;
	pResponsePdu->pPayload[1] = startAddress & 0xFF;
	pResponsePdu->pPayload[2] = (quantity >> 8) & 0xFF;
	pResponsePdu->pPayload[3] = quantity & 0xFF;
}

//------------------------------------------------------------------------------
//
static void modbus_ProcessWriteMultipleRegisters(modbus_Pdu_t *pResponsePdu, modbus_WriteCallback_t pCallback, modbus_FunctionCode_e functionCode, uint16_t startAddress, uint16_t quantity, uint8_t *pByteBuffer)
{
	MODBUS_ASSERT(pResponsePdu != NULL);
	MODBUS_ASSERT(pCallback != NULL);
	MODBUS_ASSERT(pByteBuffer != NULL);

	uint16_t valueBuffer = 0;
	modbus_Exception_e ret = 0;

	for(uint16_t registerCtr = 0; registerCtr < quantity; registerCtr++)
	{
		const uint16_t address = startAddress + registerCtr;
		valueBuffer =
			(((uint16_t)pByteBuffer[registerCtr * 2] << 8) & 0xFF00) |
			((uint16_t)pByteBuffer[registerCtr * 2 + 1] & 0x00FF);

		ret = pCallback(functionCode, address, valueBuffer);
		if(ret != MODBUS_EXCEPTION_SUCCESS)
		{
			modbus_SetExceptionResponse(ret, pResponsePdu);
			return;
		}
	}

	pResponsePdu->functionCode = functionCode;
	pResponsePdu->payloadSize = 4;
	pResponsePdu->pPayload[0] = (startAddress >> 8) & 0xFF;
	pResponsePdu->pPayload[1] = startAddress & 0xFF;
	pResponsePdu->pPayload[2] = (quantity >> 8) & 0xFF;
	pResponsePdu->pPayload[3] = quantity & 0xFF;
}


