/*
 * BMP388.c
 *
 *  Created on: Oct 28, 2024
 *      Author: LokeshTanwar
 */

#include"stm32f4xx.h"
#include"BMP388.h"
#include "math.h"


/* ----- PRIVATE FUNCTIONS PROTOTYPES ----- */
HAL_StatusTypeDef	BMP388_SoftReset(BMP388_HandleTypeDef *bmp);
HAL_StatusTypeDef	BMP388_GetCalibData(BMP388_HandleTypeDef *bmp);
            float	BMP388_CompensateTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_temp, float *temp);
            float	BMP388_CompensatePress(BMP388_HandleTypeDef *bmp, float temp, uint32_t raw_press, float *press);
HAL_StatusTypeDef	BMP388_ReadBytes(BMP388_HandleTypeDef *bmp, uint8_t reg_addr, uint8_t *buff, uint8_t len);
HAL_StatusTypeDef	BMP388_WriteBytes(BMP388_HandleTypeDef *bmp, uint8_t reg_addr, uint8_t *buff, uint8_t len);



uint8_t BMP388_Init(BMP388_HandleTypeDef *bmp, I2C_HandleTypeDef *i2cHandle){
    uint8_t check;
    HAL_StatusTypeDef state;
    bmp->i2cHandle = i2cHandle;
    state = HAL_I2C_Mem_Read(bmp->i2cHandle, BMP388_I2C_ADDR << 1, BMP388_CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000);

    if (state == HAL_OK && check == 0x50) {

    	state = BMP388_GetCalibData(bmp);
    	if(state == HAL_OK){
    		return 1;
    	}
    	else{
    		return 2;
    	}
    }else{
    	return 0;
    }

}

HAL_StatusTypeDef BMP388_SetTempOS(BMP388_HandleTypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->osr = (bmp->osr & 0xC7) | (oversample << 3); // == 0b11000111
	return HAL_OK;
}

HAL_StatusTypeDef BMP388_SetPressOS(BMP388_HandleTypeDef *bmp, uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	bmp->osr = (bmp->osr & 0xF8) | oversample; // 0xF8 == 0b11111000
	return HAL_OK;
}


HAL_StatusTypeDef BMP388_SetIIRFilterCoeff(BMP388_HandleTypeDef *bmp, uint8_t filtercoeff){
	if(filtercoeff > BMP3_IIR_FILTER_COEFF_127){
		return HAL_ERROR;
	}
	bmp->iir = filtercoeff << 1;
	return HAL_OK;
}

HAL_StatusTypeDef BMP388_SetOutputDataRate(BMP388_HandleTypeDef *bmp, uint8_t odr){
	if(odr > BMP3_ODR_0_001_HZ){
		return HAL_ERROR;
	}
	bmp->odr = odr;
	return HAL_OK;
}


void BMP388_CompensateRawPressTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_pressure, uint32_t raw_temperature){

	BMP388_CompensateTemp(bmp, raw_temperature, &(bmp->temperature));
	BMP388_CompensatePress(bmp, bmp->temperature, raw_pressure, &(bmp->pressure));

}

float BMP388_FindAltitude(float ground_pressure, float pressure){
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return 44330.0 * (1.0 - pow(pressure / ground_pressure, 0.1903));
}


HAL_StatusTypeDef BMP388_ReadRawPressTempTime(BMP388_HandleTypeDef *bmp){
	HAL_StatusTypeDef rslt;
	uint8_t pwr_ctrl = BMP3_PWR_CTRL_PRESS_ON | BMP3_PWR_CTRL_TEMP_ON | BMP3_PWR_CTRL_MODE_FORCED;

	uint8_t oversampling = bmp->osr;
	uint8_t odr = bmp->odr;
	uint8_t filtercoeff = bmp->iir;

	uint32_t raw_temperature;
	uint32_t raw_pressure;



	// Set OSR register
	rslt = BMP388_WriteBytes(bmp, OSR, &oversampling, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set ODR register
	rslt = BMP388_WriteBytes(bmp, ODR, &odr, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set CONFIG register
	rslt = BMP388_WriteBytes(bmp, CONFIG, &filtercoeff, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set PWR_CTRL register
	rslt = BMP388_WriteBytes(bmp, PWR_CTRL, &pwr_ctrl, 1);
	if(rslt != HAL_OK){
		return rslt;
	}

	uint8_t raw_data[11];
	// Get raw data for pressure and temperature
	rslt = BMP388_ReadBytes(bmp, BMP388_DATA0, raw_data, 11);
	if(rslt != HAL_OK){
		return rslt;
	}

	// Parsing pressure data
	raw_pressure = (uint32_t)raw_data[2] << 16 | (uint32_t)raw_data[1] << 8 | (uint32_t)raw_data[0];

	// Parsing temperature data
	raw_temperature = (uint32_t)raw_data[5] << 16 | (uint32_t)raw_data[4] << 8 | (uint32_t)raw_data[3];

	// Parsing time bytes
	bmp->time = (uint32_t)raw_data[10] << 16 | (uint32_t)raw_data[9] << 8 | (uint32_t)raw_data[8];

	BMP388_CompensateRawPressTemp(bmp,raw_pressure,raw_temperature);
	bmp->altitude = BMP388_FindAltitude(101325,bmp->pressure);
	return rslt;
}






HAL_StatusTypeDef BMP388_StartNormalModeFIFO(BMP388_HandleTypeDef *bmp){
	HAL_StatusTypeDef rslt;

	uint8_t pwr_ctrl = BMP3_PWR_CTRL_PRESS_ON | BMP3_PWR_CTRL_TEMP_ON | BMP3_PWR_CTRL_MODE_NORMAL;

	uint8_t fifo_config_1 = BMP3_FIFO_CONFIG_1_FIFO_MODE_ON | BMP3_FIFO_CONFIG_1_FIFO_STOP_ON_FULL_ON |
                            BMP3_FIFO_CONFIG_1_FIFO_TIME_EN_OFF | BMP3_FIFO_CONFIG_1_FIFO_PRESS_EN_ON |
							BMP3_FIFO_CONFIG_1_FIFO_TEMP_EN_ON;

	uint8_t oversampling = bmp->osr;
	uint8_t odr = bmp->odr;
	uint8_t filtercoeff = bmp->iir;



	// Set OSR register
	rslt = BMP388_WriteBytes(bmp, OSR, &oversampling, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set ODR register
	rslt = BMP388_WriteBytes(bmp, ODR, &odr, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set CONFIG register
	rslt = BMP388_WriteBytes(bmp, CONFIG, &filtercoeff, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set PWR_CTRL register
	rslt = BMP388_WriteBytes(bmp, PWR_CTRL, &pwr_ctrl, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set FIFO_CONFIG_1 register
	rslt = BMP388_WriteBytes(bmp, FIFO_CONFIG_1, &fifo_config_1, 1);
	if(rslt != HAL_OK){
		return rslt;
	}

	return rslt;
}


HAL_StatusTypeDef BMP388_GetFIFOLength(BMP388_HandleTypeDef *bmp, uint16_t *len){
	HAL_StatusTypeDef rslt;

	uint8_t raw_fifo_len[2];

	rslt = BMP388_ReadBytes(bmp, FIFO_LENGTH_0, raw_fifo_len, 2);
	if(rslt != HAL_OK){
		return rslt;
	}

	*len = raw_fifo_len[1] << 8 | raw_fifo_len[0];

	return rslt;
}

HAL_StatusTypeDef BMP388_GetRawDataFIFO(BMP388_HandleTypeDef *bmp, uint16_t bytes_num, uint8_t raw_data[]){
	HAL_StatusTypeDef rslt;


	rslt = BMP388_ReadBytes(bmp, FIFO_DATA, raw_data, bytes_num+4);
	if(rslt != HAL_OK){
		return rslt;
	}


	return rslt;
}


HAL_StatusTypeDef BMP388_SoftReset(BMP388_HandleTypeDef *bmp){
	uint8_t rst_cmnd = BMP388_SOFTRESET;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

	HAL_StatusTypeDef rslt;

	// Reading STATUS reg to understand that the BMP388 is ready to receive command
	rslt = BMP388_ReadBytes(bmp, STATUS, &cmd_rdy_status , 1);
	if((rslt == HAL_OK) && (cmd_rdy_status & BMP3_CMD_RDY)){
		// Writing SOFTRESET command to CMD reg
		rslt = BMP388_WriteBytes(bmp, CMD, &rst_cmnd, 1);
		if(rslt == HAL_OK){
			// 2 ms pause then check ERR reg
			HAL_Delay(2);
			rslt = BMP388_ReadBytes(bmp, ERR_REG, &cmd_err_status, 1);
			if((cmd_err_status & CMD) || (rslt != HAL_OK)){
				return rslt;
			}
		}
		else{
			return rslt;
		}
	}

	return rslt;
}

HAL_StatusTypeDef BMP388_GetCalibData(BMP388_HandleTypeDef *bmp){

	HAL_StatusTypeDef rslt;

	uint8_t calib_buff[21] = {0};

	uint16_t	raw_par_t1;
	uint16_t	raw_par_t2;
	int8_t		raw_par_t3;
	int16_t		raw_par_p1;
	int16_t		raw_par_p2;
	int8_t		raw_par_p3;
	int8_t		raw_par_p4;
	uint16_t	raw_par_p5;
	uint16_t	raw_par_p6;
	int8_t		raw_par_p7;
	int8_t		raw_par_p8;
	int16_t		raw_par_p9;
	int8_t		raw_par_p10;
	int8_t		raw_par_p11;


	rslt = BMP388_ReadBytes(bmp,BMP388_CalibrationData,calib_buff,21);

	float temp_var;
	if(rslt == HAL_OK){
		temp_var = 0.00390625f;
		raw_par_t1 = ((uint16_t)calib_buff[1] << 8) | (uint16_t)calib_buff[0];
		bmp->T1 = (float)raw_par_t1/temp_var;

		temp_var = 1073741824.f;
		raw_par_t2 = ((uint16_t)calib_buff[3]<<8)|(uint16_t)calib_buff[0];
		bmp->T2 = (float)raw_par_t2/temp_var;

		temp_var = 281474976710656.f;
		raw_par_t3 = calib_buff[4];
		bmp->T3 = (float)raw_par_t3/temp_var;


		// PAR_P1
		temp_var = 1048576.f;
		raw_par_p1 = ((int16_t)calib_buff[6] << 8) | (int16_t)calib_buff[5];
		bmp->P1 = ((float)raw_par_p1 - 16384) / temp_var;
		// PAR_P2
		temp_var = 536870912.f;
		raw_par_p2 = ((int16_t)calib_buff[8] << 8) | (int16_t)calib_buff[7];
		bmp->P2 = ((float)raw_par_p2 - 16384) / temp_var;
		// PAR_P3
		temp_var = 4294967296.f;
		raw_par_p3 = (int8_t)calib_buff[9];
		bmp->P3 = (float)raw_par_p3 / temp_var;
		// PAR_P4
		temp_var = 137438953472.f;
		raw_par_p4 = (int8_t)calib_buff[10];
		bmp->P4 = (float)raw_par_p4 / temp_var;
		// PAR_P5
		temp_var = 0.125f;
		raw_par_p5 = ((uint16_t)calib_buff[12] << 8) | (uint16_t)calib_buff[11];
		bmp->P5 = (float)raw_par_p5 / temp_var;
		// PAR_P6
		temp_var = 64.f;
		raw_par_p6 = ((uint16_t)calib_buff[14] << 8) | (uint16_t)calib_buff[13];
		bmp->P6 = (float)raw_par_p6 / temp_var;
		// PAR_P7
		temp_var = 256.f;
		raw_par_p7 = (int8_t)calib_buff[15];
		bmp->P7 = (float)raw_par_p7 / temp_var;
		// PAR_P8
		temp_var = 32768.f;
		raw_par_p8 = (int8_t)calib_buff[16];
		bmp->P8 = (float)raw_par_p8 / temp_var;
		// PAR_P9
		temp_var = 281474976710656.f;
		raw_par_p9 = ((int16_t)calib_buff[18] << 8) | (int16_t)calib_buff[17];
		bmp->P9 = (float)raw_par_p9 / temp_var;
		// PAR_P10
		temp_var = 281474976710656.f;
		raw_par_p10 = (int8_t)calib_buff[19];
		bmp->P10 = (float)raw_par_p10 / temp_var;
		// PAR_P11
		temp_var = 36893488147419103232.f;
		raw_par_p11 = (int8_t)calib_buff[20];
		bmp->P11 = (float)raw_par_p11 / temp_var;
	}

	return rslt;

}




float BMP388_CompensateTemp(BMP388_HandleTypeDef *bmp, uint32_t raw_temp, float *temp){
    float partial_data1 = (float)(raw_temp - bmp->T1);;
    float partial_data2 = (float)(partial_data1 * bmp->T2);

    *temp = partial_data2 + (partial_data1 * partial_data1) * bmp->T3;

    return *temp;
}

float BMP388_CompensatePress(BMP388_HandleTypeDef *bmp, float temp, uint32_t raw_press, float *press){
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;



    partial_data1 = bmp->P6* temp;
    partial_data2 = bmp->P7 * (temp * temp);
    partial_data3 = bmp->P8 * (temp * temp * temp);
    partial_out1 = bmp->P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = bmp->P2 * temp;
    partial_data2 = bmp->P3 * (temp * temp);
    partial_data3 = bmp->P4 * (temp * temp * temp);
    partial_out2 = (float)raw_press * (bmp->P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = bmp->P9 + bmp->P10 * temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * bmp->P11;

    *press = partial_out1 + partial_out2 + partial_data4;

    return *press;
}


 HAL_StatusTypeDef BMP388_ReadBytes(BMP388_HandleTypeDef *bmp,uint8_t reg_addr, uint8_t *buff, uint8_t len){
 	return HAL_I2C_Mem_Read(bmp->i2cHandle, BMP388_I2C_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
 }



 /*!
  *  @brief Function to write byte from BMP388 in blocking mode
  *
  *	@param[in] bmp			: Pointer to BMP388 structure
  *  @param[in] reg_addr     : Register address.
  *  @param[out] buff	    : Pointer to the data buffer to store the read data.
  *  @param[in] len          : Amount of bytes to write.
  *
  *  @return Status of execution
  *  @retval = HAL_OK 		-> Success
  *  @retval != HAL_ERROR 	-> Failure Info
  */
 HAL_StatusTypeDef BMP388_WriteBytes(BMP388_HandleTypeDef *bmp, uint8_t reg_addr, uint8_t *buff, uint8_t len){
 	return HAL_I2C_Mem_Write(bmp->i2cHandle, BMP388_I2C_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
 }




