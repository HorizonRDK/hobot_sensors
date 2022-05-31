#ifndef __TOF_ERROR_H__
#define __TOF_ERROR_H__


typedef enum tagTOFRET
{
	/** Success (no error) */
	TOFRET_SUCCESS = 0x00000000,
	/** Success (no error, and start to read calib data) */
	TOFRET_SUCCESS_READING_CALIB = 0x00000001,



	 /** Input/output error */
	TOFRET_ERROR_IO = 0x80000001,
	 /** Invalid parameter */
	TOFRET_ERROR_INVALID_PARAM = 0x80000002,
	/** Access denied (insufficient permissions) */
	TOFRET_ERROR_ACCESS = 0x80000003,
	/** No such device (it may have been disconnected) */
	TOFRET_ERROR_NO_DEVICE = 0x80000004,
	/** Operation timed out */
	TOFRET_ERROR_TIMEOUT = 0x80000005,
	/** Overflow */
	TOFRET_ERROR_OVERFLOW = 0x80000006,
	/** Insufficient memory */
	TOFRET_ERROR_NO_MEM = 0x80000007,
	/** Operation not supported or unimplemented on this platform */
	TOFRET_ERROR_WRONG_STATUS	= 0x80000008,
	/** Operation not supported */
	TOFRET_ERROR_NOT_SUPPORTED = 0x80000009,
	/** Device is in use now */
	TOFRET_ERROR_ALREADY_IN_USE = 0x8000000A,
	/** Error Data */
	TOFRET_ERROR_DATA = 0x8000000B,
	/** Cfg file not found */
	TOFRET_ERROR_CFG_FILE_NOT_FOUND = 0x8000000C,
	/** Read Calib failed */
	TOFRET_ERROR_READ_CALIB_FAILED = 0x8000000D,

	/** USB write error */
	TOFRET_ERROR_USB_WRITE = 0x80010001,
	/** USB read error */
	TOFRET_ERROR_USB_READ = 0x80010002,
	/** USB disconnect */
	TOFRET_ERROR_USB_DISCONNECT = 0x80010003,

	/* generic fail */
	TOFRET_HAL_FAILED = 0x80060001,
	/* operation not support */
	TOFRET_HAL_UNSUPPORT = 0x80060002,
	/* device is unreponsive */
	TOFRET_HAL_HARDWARE_UNRESPONSIVE = 0x80060003,
	/* timeout */
	TOFRET_HAL_TIMEOUT = 0x80060004,
	/* interface board not support */
	TOFRET_HAL_INTERFACE_BOARD_NOT_SUPPORT = 0x80060005,
	/* configuration read error */
	TOFRET_HAL_CONFIG_READ_FAILED = 0x80060006,
	/* module dll load failed */
	TOFRET_HAL_MODULE_LOAD_FAILED = 0x80060007,
	/* call module dll function failed */
	TOFRET_HAL_MODULE_SYSMBOL_CALL_FAILED = 0x80060008,
	/* object instance failed */
	TOFRET_HAL_OBJ_INSTANCE_FAILED = 0x80060009,
	/* not found camera */
	TOFRET_HAL_CAMERA_NOT_FOUND = 0x8006000A,
	/* platform setting failed */
	TOFRET_HAL_INTERFACE_BOARD_SETTING_FAILED = 0x8006000B,
	/* iic read failed */
	TOFRET_HAL_IIC_READ_FAILED = 0x8006000C,
	/* iic write failed */
	TOFRET_HAL_IIC_WRITE_FAILED = 0x8006000D,
	/* io operation failed */
	TOFRET_HAL_IO_SETTING_FAILED = 0x8006000E,
	
	/** Other error */
	TOFRET_ERROR_OTHER = 0x88100001,
}TOFRET;


#endif

