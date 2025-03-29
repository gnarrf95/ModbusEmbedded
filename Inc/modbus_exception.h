
#ifndef __INCLUDE_MODBUS_EXCEPTION_H
#define __INCLUDE_MODBUS_EXCEPTION_H

#ifdef __cplusplus
extern "C" {
#endif



typedef enum
{
	MODBUS_EXCEPTION_SUCCESS                    = 0x00,

	MODBUS_EXCEPTION_ILLEGALFUNCTION            = 0x01,
	MODBUS_EXCEPTION_ILLEGALDATAADDRESS         = 0x02,
	MODBUS_EXCEPTION_ILLEGALDATAVALUE           = 0x03,
	MODBUS_EXCEPTION_SLAVEDEVICEFAILURE         = 0x04,
	MODBUS_EXCEPTION_ACKNOWLEDGE                = 0x05,
	MODBUS_EXCEPTION_SLAVEDEVICEBUSY            = 0x06,
	MODBUS_EXCEPTION_MEMORYPARITYERROR          = 0x08,
	MODBUS_EXCEPTION_GATEWAYPATHUNAVAILABLE     = 0x0A,
	MODBUS_EXCEPTION_GATEWAYDEVICEFAILEDTORESP  = 0x0B
} modbus_Exception_e;



#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MODBUS_EXCEPTION_H */
