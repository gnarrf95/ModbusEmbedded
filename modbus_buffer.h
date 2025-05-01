
#ifndef __INCLUDE_MODBUS_BUFFER_H
#define __INCLUDE_MODBUS_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <ModbusEmbedded/modbus.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef enum
{
	MODBUS_BUFFER_ACCESS_READONLY = 0,
	MODBUS_BUFFER_ACCESS_WRITEONLY,
	MODBUS_BUFFER_ACCESS_READWRITE,

	MODBUS_BUFFER_ACCESS_LIMIT
} modbus_Buffer_Access_e;

typedef struct
{
	uint16_t startAddress;
	modbus_Buffer_Access_e accessType;

	uint8_t *pDataBuffer;
	uint32_t dataSizeBytes;
} modbus_Buffer_Datapoint_t;

typedef struct
{
	const modbus_Buffer_Datapoint_t *pArray;
	const uint32_t arraySize;
} modbus_Buffer_t;



modbus_Exception_e modbus_Buffer_ReadRegister(const modbus_Buffer_t *pBuffer, uint16_t registerAddress, uint16_t *pRegisterBuffer);
modbus_Exception_e modbus_Buffer_WriteRegister(const modbus_Buffer_t *pBuffer, uint16_t registerAddress, uint16_t registerValue);



#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MODBUS_BUFFER_H */
