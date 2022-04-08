/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_UTILS_H_
#define X3_UTILS_H_

typedef enum {
	E_CHIP_X3M,
	E_CHIP_X3E,
	E_CHIP_UNKNOW,
} E_CHIP_TYPE;


#define SENSOR_F37_SUPPORT			1
#define SENSOR_IMX415_SUPPORT		2
#define SENSOR_OS8A10_SUPPORT		4
#define SENSOR_OV8856_SUPPORT		8
#define SENSOR_SC031GS_SUPPORT		16

typedef struct {
	E_CHIP_TYPE m_chip_type; // 芯片类型
	unsigned int m_sensor_list; // 每个bit对应一种sensor
}hard_capability_t; // 硬件能力

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

E_CHIP_TYPE x3_get_chip_type(void);
int x3_get_hard_capability(hard_capability_t *capability);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_UTILS_H_