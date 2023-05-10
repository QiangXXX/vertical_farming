#ifndef _BME680_H_
#define _BME680_H_

/* BME680 REGISTERS */

#define BME680_STATUS       0x73
#define BME680_RESET        0xE0
#define BME680_ID           0xD0
#define BME680_CONFIG       0x75
#define BME680_CTRL_MEAS    0x74
#define BME680_CTRL_HUM     0x72 
#define BME680_CTRL_GAS_1   0x71
#define BME680_CTRL_GAS_0   0x70

#define BME680_GAS_WAIT_0   0x64 
#define BME680_GAS_WAIT_1   0x65 
#define BME680_GAS_WAIT_2   0x66 
#define BME680_GAS_WAIT_3   0x67 
#define BME680_GAS_WAIT_4   0x68 
#define BME680_GAS_WAIT_5   0x69 
#define BME680_GAS_WAIT_6   0x6A 
#define BME680_GAS_WAIT_7   0x6B 
#define BME680_GAS_WAIT_8   0x6C 
#define BME680_GAS_WAIT_9   0x6D 

#define BME680_RES_HEAT_0   0x5A
#define BME680_RES_HEAT_1   0x5B
#define BME680_RES_HEAT_2   0x5C
#define BME680_RES_HEAT_3   0x5D
#define BME680_RES_HEAT_4   0x5E
#define BME680_RES_HEAT_5   0x5F
#define BME680_RES_HEAT_6   0x60
#define BME680_RES_HEAT_7   0x61
#define BME680_RES_HEAT_8   0x62
#define BME680_RES_HEAT_9   0x63

#define BME680_IDAC_HEAD_0  0x50
#define BME680_IDAC_HEAD_1  0x51
#define BME680_IDAC_HEAD_2  0x52
#define BME680_IDAC_HEAD_3  0x53
#define BME680_IDAC_HEAD_4  0x54
#define BME680_IDAC_HEAD_5  0x55
#define BME680_IDAC_HEAD_6  0x56
#define BME680_IDAC_HEAD_7  0x57
#define BME680_IDAC_HEAD_8  0x58
#define BME680_IDAC_HEAD_9  0x59

#define BME680_GAS_R_LSB    0x2B 
#define BME680_GAS_R_MSB    0x2A 
#define BME680_HUM_LSB      0x26
#define BME680_HUM_MSB      0x25 
#define BME680_TEMP_XLSB    0x24
#define BME680_TEMP_LSB     0x23 
#define BME680_TEMP_MSB     0x22 
#define BME680_PRESS_XLSB   0x21 
#define BME680_PRESS_LSB    0x20
#define BME680_PRESS_MSB    0x1F 
#define BME680_EAS_STATUS_0 0x1D

#endif /* _BME680_H_ */
