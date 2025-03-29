
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <modbus.h>



static inline void modbus_byte_to_hexstr(uint8_t value, char *pBuffer)
{
	uint8_t digit = (value >> 4) & 0x0F;
	if(digit >= 0x0A)
	{
		pBuffer[0] = (digit - 0x0A) + 'A';
	}
	else
	{
		pBuffer[0] = digit + '0';
	}

	digit = value & 0x0F;
	if(digit >= 0x0A)
	{
		pBuffer[1] = (digit - 0x0A) + 'A';
	}
	else
	{
		pBuffer[1] = digit + '0';
	}
}



//------------------------------------------------------------------------------
//
uint16_t modbus_EncodeAscii(char *pBuffer, uint16_t bufferSize, modbus_Pdu_t *pPdu)
{
	MODBUS_ASSERT(pBuffer != NULL);
	MODBUS_ASSERT(pPdu != NULL);

	if(bufferSize < (pPdu->payloadSize + 9))
	{
		// Buffer too small for PDU
		return 0;
	}

	char hexstring_buf[2] = { 0 };
	pBuffer[0] = ':';

	// Address
	modbus_byte_to_hexstr(pPdu->busAddress, (char *)hexstring_buf);
	pBuffer[1] = hexstring_buf[0];
	pBuffer[2] = hexstring_buf[1];

	// Function Code
	modbus_byte_to_hexstr((uint8_t)pPdu->functionCode, (char *)hexstring_buf);
	pBuffer[3] = hexstring_buf[0];
	pBuffer[4] = hexstring_buf[1];

	// Payload
	for(uint16_t ctr = 0; ctr < pPdu->payloadSize; ctr++)
	{
		modbus_byte_to_hexstr(pPdu->pPayload[ctr], (char *)hexstring_buf);
		pBuffer[2*ctr + 5] = hexstring_buf[0];
		pBuffer[2*ctr + 6] = hexstring_buf[1];
	}

	// LRC Checksum
	modbus_byte_to_hexstr(modbus_GenerateLrc(pPdu), (char *)hexstring_buf);
	pBuffer[pPdu->payloadSize*2 + 5] = hexstring_buf[0];
	pBuffer[pPdu->payloadSize*2 + 6] = hexstring_buf[1];

	// Stop Characters
	pBuffer[pPdu->payloadSize*2 + 7] = '\r';
	pBuffer[pPdu->payloadSize*2 + 8] = '\n';

	return (pPdu->payloadSize*2 + 9);
}

//------------------------------------------------------------------------------
//
bool modbus_DecodeAscii(const char *pData, uint16_t dataSize, modbus_Pdu_t *pPdu)
{
	MODBUS_ASSERT(pData != NULL);
	MODBUS_ASSERT(pPdu != NULL);

	if(dataSize < 9)
	{
		return false;
	}

	if(
		(pData[0] != ':') ||
		(pData[dataSize - 2] != '\r') ||
		(pData[dataSize - 1] != '\n')
	)
	{
		// Invalid Format.
		return false;
	}

	char hexstring_buf[3] = { 0 };
	uint32_t byte_buf = 0;

	// Checksum
	hexstring_buf[0] = pData[dataSize - 4];
	hexstring_buf[1] = pData[dataSize - 3];
	byte_buf = strtoul((char *)hexstring_buf, NULL, 16);
	uint8_t checksum = byte_buf & 0xFF;

	// Address
	hexstring_buf[0] = pData[1];
	hexstring_buf[1] = pData[2];
	byte_buf = strtoul((char *)hexstring_buf, NULL, 16);
	pPdu->busAddress = byte_buf & 0xFF;

	// Function Code
	hexstring_buf[0] = pData[3];
	hexstring_buf[1] = pData[4];
	byte_buf = strtoul((char *)hexstring_buf, NULL, 16);
	pPdu->functionCode = (modbus_FunctionCode_e)(byte_buf & 0xFF);

	// Payload
	pPdu->payloadSize = 0;
	for(uint16_t ctr = 5; ctr < (dataSize - 4); ctr += 2)
	{
		hexstring_buf[0] = pData[ctr];
		hexstring_buf[1] = pData[ctr+1];
		byte_buf = strtoul(hexstring_buf, NULL, 16);

		pPdu->pPayload[pPdu->payloadSize] = (uint8_t)byte_buf;
		pPdu->payloadSize++;
	}

	if(modbus_GenerateLrc(pPdu) != checksum)
	{
		// Checksum does not match.
		return false;
	}

	return true;
}

//------------------------------------------------------------------------------
//
uint16_t modbus_EncodeRtu(uint8_t *pBuffer, uint16_t bufferSize, modbus_Pdu_t *pPdu)
{
	MODBUS_ASSERT(pBuffer != NULL);
	MODBUS_ASSERT(pPdu != NULL);

	if(bufferSize < (pPdu->payloadSize + 4))
	{
		// Buffer too small for PDU
		return 0;
	}

	pBuffer[0] = pPdu->busAddress;
	pBuffer[1] = (uint8_t)pPdu->functionCode;

	for(uint16_t ctr = 0; ctr < pPdu->payloadSize; ctr++)
	{
		pBuffer[ctr+2] = pPdu->pPayload[ctr];
	}

	uint16_t checksum = modbus_GenerateCrc(pPdu);
	pBuffer[pPdu->payloadSize+2] = checksum & 0xFF;
	pBuffer[pPdu->payloadSize+3] = (checksum >> 8) & 0xFF;

	return (pPdu->payloadSize + 4);
}

//------------------------------------------------------------------------------
//
bool modbus_DecodeRtu(const uint8_t *pData, uint16_t dataSize, modbus_Pdu_t *pPdu)
{
	MODBUS_ASSERT(pData != NULL);
	MODBUS_ASSERT(pPdu != NULL);

	if(dataSize < 5)
	{
		return false;
	}

	pPdu->payloadSize = dataSize - 4;
	pPdu->busAddress = pData[0];
	pPdu->functionCode = (modbus_FunctionCode_e)pData[1];

	uint16_t checksum = (uint16_t)pData[dataSize-2] | ((uint16_t)pData[dataSize-1] << 8);

	for(uint16_t ctr = 0; ctr < (dataSize-4); ctr++)
	{
		pPdu->pPayload[ctr] = pData[ctr+2];
	}

	if(modbus_GenerateCrc(pPdu) != checksum)
	{
		// Checksum does not match.
		return false;
	}

	return true;
}
