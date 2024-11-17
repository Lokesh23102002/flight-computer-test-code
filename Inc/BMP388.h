/*
 * BMP388.h
 *
 *  Created on: Oct 28, 2024
 *      Author: LokeshTanwar
 */

#ifndef INC_BMP388_H_
#define INC_BMP388_H_


/* BMP 388 commands */
#define BMP3_CMD_RDY                                    0x10
#define BMP388_SOFTRESET                                0xB6

/* Over sampling macros */
#define BMP388_NO_OVERSAMPLING                          0x00
#define BMP388_OVERSAMPLING_2X                          0x01
#define BMP388_OVERSAMPLING_4X                          0x02
#define BMP388_OVERSAMPLING_8X                          0x03
#define BMP388_OVERSAMPLING_16X                         0x04
#define BMP388_OVERSAMPLING_32X                         0x05

/* Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE                         0x00
#define BMP3_IIR_FILTER_COEFF_1                         0x01
#define BMP3_IIR_FILTER_COEFF_3                         0x02
#define BMP3_IIR_FILTER_COEFF_7                         0x03
#define BMP3_IIR_FILTER_COEFF_15                        0x04
#define BMP3_IIR_FILTER_COEFF_31                        0x05
#define BMP3_IIR_FILTER_COEFF_63                        0x06
#define BMP3_IIR_FILTER_COEFF_127                       0x07

/* Output Data Rate macros */
#define BMP3_ODR_200_HZ                                 0x00
#define BMP3_ODR_100_HZ                                 0x01
#define BMP3_ODR_50_HZ                                  0x02
#define BMP3_ODR_25_HZ                                  0x03
#define BMP3_ODR_12_5_HZ                                0x04
#define BMP3_ODR_6_25_HZ                                0x05
#define BMP3_ODR_3_1_HZ                                 0x06
#define BMP3_ODR_1_5_HZ                                 0x07
#define BMP3_ODR_0_78_HZ                                0x08
#define BMP3_ODR_0_39_HZ                                0x09
#define BMP3_ODR_0_2_HZ                                 0x0A
#define BMP3_ODR_0_1_HZ                                 0x0B
#define BMP3_ODR_0_05_HZ                                0x0C
#define BMP3_ODR_0_02_HZ                                0x0D
#define BMP3_ODR_0_01_HZ                                0x0E
#define BMP3_ODR_0_006_HZ                               0x0F
#define BMP3_ODR_0_003_HZ                               0x10
#define BMP3_ODR_0_001_HZ                               0x11

#define BMP3_CALIBDATA_LEN                              21

/* FIFO */
#define BMP3_NORMAL_PRESS_AND_TEMP_FRAME_HEADER         0x94U
#define BMP3_SENSOR_TIME_FRAME_HEADER                   0xA0U
#define BMP3_EMPTY_FRAME_HEADER                         0x80U


/* ----- REGISTER MACROS ----- */

#define BMP3_PWR_CTRL_PRESS_ON                          1U
#define BMP3_PWR_CTRL_PRESS_OFF                         0U
#define BMP3_PWR_CTRL_TEMP_ON                           (1U << 1)
#define BMP3_PWR_CTRL_TEMP_OFF                          0U
#define BMP3_PWR_CTRL_MODE_SLEEP                        0U
#define BMP3_PWR_CTRL_MODE_FORCED                       (1U << 4)
#define BMP3_PWR_CTRL_MODE_NORMAL                       (0x03U << 4)

#define BMP3_FIFO_CONFIG_1_FIFO_MODE_ON                 1U
#define BMP3_FIFO_CONFIG_1_FIFO_MODE_OFF                0U
#define BMP3_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_ON         (1U << 1)
#define BMP3_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_OFF        0U
#define BMP3_FIFO_CONFIG_1_FIFO_TIME_EN_ON              (1U << 2)
#define BMP3_FIFO_CONFIG_1_FIFO_TIME_EN_OFF             0U
#define BMP3_FIFO_CONFIG_1_FIFO_PRESS_EN_ON             (1U << 3)
#define BMP3_FIFO_CONFIG_1_FIFO_PRESS_EN_OFF            0U
#define BMP3_FIFO_CONFIG_1_FIFO_TEMP_EN_ON              (1U << 4)
#define BMP3_FIFO_CONFIG_1_FIFO_TEMP_EN_OFF             0U


#define	CHIP_ID             0x00
#define ERR_REG             0x02
#define STATUS              0x03
#define	INT_STATUS          0x11
#define FIFO_LENGTH_0       0x12
#define FIFO_LENGTH_1      	0x13
#define FIFO_DATA          	0x14
#define FIFO_WTM_0         	0x15
#define FIFO_WTM_1         	0x16
#define FIFO_CONFIG_1      	0x17
#define FIFO_CONFIG_2      	0x18
#define INT_CTRL           	0x19
#define IF_CONF            	0x1A
#define PWR_CTRL           	0x1B
#define OSR                	0x1C
#define ODR                	0x1D
#define CONFIG             	0x1F
#define CALIB_DATA         	0x31
#define CMD                	0x7E

#define BMP388_I2C_ADDR     0x76  // BMP388 default I2C address (0x76)
#define BMP388_CHIP_ID_REG  0x00
#define BMP388_DATA0  0x04
#define BMP388_CalibrationData       0x31


typedef struct {
	I2C_HandleTypeDef *i2cHandle; // Pointer to the I2C port (type void* for flexibility)


//  Various sensor variables and constant
	float pressure;
	float temperature;
	float altitude;
	uint32_t time;

	uint8_t osr;
	uint8_t iir;
	uint8_t odr;


	float T1;
	float T2;
	float T3;
	float P1;
	float P2;
	float P3;
	float P4;
	float P5;
	float P6;
	float P7;
	float P8;
	float P9;
	float P10;
	float P11;
	float T_fine;

} BMP388_HandleTypeDef;


uint8_t BMP388_Init(BMP388_HandleTypeDef *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388_HandleTypeDef *bmp);




#endif /* INC_BMP388_H_ */
