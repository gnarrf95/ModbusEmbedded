
#ifndef __INCLUDE_MODBUS_DEFS_H
#define __INCLUDE_MODBUS_DEFS_H

#include <stdint.h>
#include <ModbusEmbedded/modbus_function.h>

#ifdef __cplusplus
extern "C" {
#endif



#define MODBUS_PDU_SIZE         253
#define MODBUS_PAYLOAD_SIZE     (MODBUS_PDU_SIZE)



typedef struct
{
    uint8_t busAddress;
    modbus_FunctionCode_e functionCode;

    uint8_t pPayload[MODBUS_PAYLOAD_SIZE];
    uint16_t payloadSize;
} modbus_Pdu_t;



#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MODBUS_DEFS_H */
