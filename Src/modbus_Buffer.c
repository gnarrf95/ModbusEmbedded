
#include <ModbusEmbedded/modbus_Buffer.h>
#include <stddef.h>



static const modbus_Buffer_Datapoint_t *modbus_Buffer_GetDatapoint(const modbus_Buffer_t *pBuffer, uint16_t registerAddress);



//------------------------------------------------------------------------------
//
modbus_Exception_e modbus_Buffer_ReadRegister(const modbus_Buffer_t *pBuffer, uint16_t registerAddress, uint16_t *pRegisterBuffer)
{
	if ((pBuffer == NULL) || (pBuffer->pArray == NULL) || (pRegisterBuffer == NULL))
	{
		return MODBUS_EXCEPTION_SLAVEDEVICEFAILURE;
	}

	const modbus_Buffer_Datapoint_t *pDatapoint = modbus_Buffer_GetDatapoint(pBuffer, registerAddress);
	if (pDatapoint == NULL)
	{
		return MODBUS_EXCEPTION_ILLEGALDATAADDRESS;
	}
	if (pDatapoint->accessType >= MODBUS_BUFFER_ACCESS_LIMIT)
	{
		return MODBUS_EXCEPTION_SLAVEDEVICEFAILURE;
	}
	if (pDatapoint->accessType == MODBUS_BUFFER_ACCESS_WRITEONLY)
	{
		return MODBUS_EXCEPTION_ILLEGALDATAADDRESS;
	}

	const uint32_t registerIndex = registerAddress - pDatapoint->startAddress;
	const uint32_t byteOffset = (pDatapoint->dataSizeBytes - 2) - (registerIndex * 2);

	*pRegisterBuffer = ((uint16_t)pDatapoint->pDataBuffer[byteOffset+1] << 8) | (uint16_t)pDatapoint->pDataBuffer[byteOffset];

	return MODBUS_EXCEPTION_SUCCESS;
}

//------------------------------------------------------------------------------
//
modbus_Exception_e modbus_Buffer_WriteRegister(const modbus_Buffer_t *pBuffer, uint16_t registerAddress, uint16_t registerValue)
{
	if ((pBuffer == NULL) || (pBuffer->pArray == NULL))
	{
		return MODBUS_EXCEPTION_SLAVEDEVICEFAILURE;
	}

	const modbus_Buffer_Datapoint_t *pDatapoint = modbus_Buffer_GetDatapoint(pBuffer, registerAddress);
	if (pDatapoint == NULL)
	{
		return MODBUS_EXCEPTION_ILLEGALDATAADDRESS;
	}
	if (pDatapoint->accessType >= MODBUS_BUFFER_ACCESS_LIMIT)
	{
		return MODBUS_EXCEPTION_SLAVEDEVICEFAILURE;
	}
	if (pDatapoint->accessType == MODBUS_BUFFER_ACCESS_READONLY)
	{
		return MODBUS_EXCEPTION_ILLEGALDATAADDRESS;
	}

	const uint32_t registerIndex = registerAddress - pDatapoint->startAddress;
	const uint32_t byteOffset = (pDatapoint->dataSizeBytes - 2) - (registerIndex * 2);

	pDatapoint->pDataBuffer[byteOffset] = (uint8_t)(registerValue & 0x00FF);
	pDatapoint->pDataBuffer[byteOffset+1] = (uint8_t)((registerValue >> 8) & 0x00FF);

	return MODBUS_EXCEPTION_SUCCESS;
}



//------------------------------------------------------------------------------
//
__attribute__ ((optimize("-Ofast")))
static const modbus_Buffer_Datapoint_t *modbus_Buffer_GetDatapoint(const modbus_Buffer_t *pBuffer, uint16_t registerAddress)
{
	if ((pBuffer == NULL) || (pBuffer->pArray == NULL))
	{
		return NULL;
	}

	for (uint32_t ctr = 0; ctr < pBuffer->arraySize; ctr++)
	{
		const uint16_t startAddress = pBuffer->pArray[ctr].startAddress;
		const uint16_t endAddress = (pBuffer->pArray[ctr].dataSizeBytes / 2) + startAddress - 1;

		if ((registerAddress >= startAddress) && (registerAddress <= endAddress))
		{
			return &pBuffer->pArray[ctr];
		}
	}

	return NULL;
}


