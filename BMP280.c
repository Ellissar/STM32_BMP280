/*
 * BMP280.c
 *
 *  Created on: 20 нояб. 2019 г.
 *      Author: ellis
 */

#include "BMP280.h"

//Чтение необработанных значений температуры и давления
void BMP280_ReadRawData (BMP280_TypeDef *BMP280)
{
	uint8_t data[6] = {0};
	HAL_I2C_Mem_Read (BMP280->hi2c, BMP280->Address, BMP280_PRESS_MSB, 1, data, 6, 100);
	BMP280->RAWPress = ((data[0]<<16) | (data[1]<<8) | data[2]) >> (9-BMP280->osrs_p);
	BMP280->RAWTemp = ((data[3]<<16) | (data[4]<<8) | data[5]) >> (9-BMP280->osrs_t);
}

//Чтение калибровочных коэффициентов
void BMP280_ReadCompensation (BMP280_TypeDef *BMP280)
{
	uint8_t data[24] = {0};
	HAL_I2C_Mem_Read (BMP280->hi2c, BMP280->Address, BMP280_DIG_T1_LSB, 1, data, 24, 100);

	BMP280->Comp.dig_T1 = (data[1]  << 8) | data[0];
	BMP280->Comp.dig_T2 = (data[3]  << 8) | data[2];
	BMP280->Comp.dig_T3 = (data[5]  << 8) | data[4];
	BMP280->Comp.dig_P1 = (data[7]  << 8) | data[6];
	BMP280->Comp.dig_P2 = (data[9]  << 8) | data[8];
	BMP280->Comp.dig_P3 = (data[11] << 8) | data[10];
	BMP280->Comp.dig_P4 = (data[13] << 8) | data[12];
	BMP280->Comp.dig_P5 = (data[15] << 8) | data[14];
	BMP280->Comp.dig_P6 = (data[17] << 8) | data[16];
	BMP280->Comp.dig_P7 = (data[19] << 8) | data[18];
	BMP280->Comp.dig_P8 = (data[21] << 8) | data[20];
	BMP280->Comp.dig_P9 = (data[23] << 8) | data[22];
}

//Начальная инициализация датчика BMP280.
//Необходимо вызвать один раз перед работой с датчиком.
HAL_StatusTypeDef BMP280_Init (BMP280_TypeDef *BMP280)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read (BMP280->hi2c, BMP280->Address, BMP280_ID, 1, &data, 1, 100);
	if (data == 0x58){
		data = (BMP280->osrs_t << 5) | (BMP280->osrs_p << 2) | BMP280->mode;
		HAL_I2C_Mem_Write (BMP280->hi2c, BMP280->Address, BMP280_CTRL_MEAS, 1, &data, 1, 100);

		data = (BMP280->t_sb << 5) | (BMP280->IIRfilter << 2);
		HAL_I2C_Mem_Write (BMP280->hi2c, BMP280->Address, BMP280_CONFIG, 1, &data, 1, 100);

		BMP280->PressDouble = 0.0;
		BMP280->PressUint32 = 0;
		BMP280->RAWPress = 0;
		BMP280->RAWTemp = 0;
		BMP280->TempDouble = 0.0;
		BMP280->TempInt32 = 0;
		BMP280->mmHgUint32 = 0;
		BMP280->mmHgDouble = 0;

		return HAL_OK;
	}
	return HAL_ERROR;
}

//Вычисление атмосферного давления и температуры
//Предполагается, что давление и температура (RAWTemp, RAWPress) будут в 20 битном формате (BMP280_OSRS_P_X16, BMP280_OSRS_T_X16)
void BMP280_GetDataReal (BMP280_TypeDef *BMP280)
{
	double var1, var2, P;
	int32_t t_fine;

	BMP280_ReadCompensation (BMP280);
	BMP280_ReadRawData (BMP280);

	//Вычисление температуры
	var1 = (((double)BMP280->RAWTemp)/16384.0 - ((double)BMP280->Comp.dig_T1)/1024.0) * ((double)BMP280->Comp.dig_T2);
	var2 = ((((double)BMP280->RAWTemp)/131072.0 - ((double)BMP280->Comp.dig_T1)/8192.0) * (((double)BMP280->RAWTemp)/131072.0 - ((double)BMP280->Comp.dig_T1)/8192.0)) * ((double)BMP280->Comp.dig_T3);
	t_fine = (int32_t)(var1 + var2);
	BMP280->TempDouble = (var1 + var2) / 5120.0;

	//Вычисление давления
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)BMP280->Comp.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)BMP280->Comp.dig_P5) * 2.0;
	var2 = (var2/4.0) + (((double)BMP280->Comp.dig_P4) * 65536.0);
	var1 = (((double)BMP280->Comp.dig_P3) * var1 * var1 / 524288.0 + ((double)BMP280->Comp.dig_P2) * var1) / 534288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)BMP280->Comp.dig_P1);
	if (var1 == 0.0){
		BMP280->PressDouble = 0;
		return; //для того, чтобы избежать исключения "деление на ноль"
	}
	P = 1048576.0 - (double)BMP280->RAWPress;
	P = (P - (var2/4096.0)) * 6250.0 / var1;
	var1 = ((double)BMP280->Comp.dig_P9) * P * P / 2147483648.0;
	var2 = P * ((double)BMP280->Comp.dig_P8) / 32768.0;
	BMP280->PressDouble = P + (var1 + var2 +((double)BMP280->Comp.dig_P7)) / 16.0;
}

//Вычисление атмосферного давления и температуры
//Предполагается, что давление и температура (RAWTemp, RAWPress) будут в 20 битном формате (BMP280_OSRS_P_X16, BMP280_OSRS_T_X16)
void BMP280_GetDataInt (BMP280_TypeDef *BMP280)
{
	int32_t var1, var2, t_fine;
	uint32_t P;

	BMP280_ReadCompensation (BMP280);
	BMP280_ReadRawData (BMP280);

	//Вычисление температуры
	var1 = (((BMP280->RAWTemp>>3) - ((int32_t)BMP280->Comp.dig_T1<<1)) * ((int32_t)BMP280->Comp.dig_T2)) >> 11;
	var2 = (((((BMP280->RAWTemp>>4) - ((int32_t)BMP280->Comp.dig_T1)) * ((BMP280->RAWTemp>>4) - ((int32_t)BMP280->Comp.dig_T1))) >> 12) * ((int32_t)BMP280->Comp.dig_T3)) >> 14;
	t_fine = var1 + var2;
	BMP280->TempInt32 = (t_fine * 5 + 128) >> 8;

	//Вычисление давления
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)BMP280->Comp.dig_P6);
	var2 = var2 + ((var1*((int32_t)BMP280->Comp.dig_P5)) << 1);
	var2 = (var2>>2) + (((int32_t)BMP280->Comp.dig_P4) << 16);
	var1 = (((BMP280->Comp.dig_P3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)BMP280->Comp.dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)BMP280->Comp.dig_P1)) >> 15);
	if (var1 == 0){
		BMP280->PressUint32 = 0;
		return; //для того, чтобы избежать исключения "деление на ноль"
	}
	P = (((uint32_t)(((int32_t)1048576) - BMP280->RAWPress) - (var2>>12))) * 3125;
	if (P < 0x80000000){
		P = (P<<1) / ((uint32_t)var1);
	}
	else {
		P = (P / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)BMP280->Comp.dig_P9) * ((int32_t)(((P>>3) * (P>>3)) >> 13))) >> 12;
	var2 = (((int32_t)(P>>2)) * ((int32_t)BMP280->Comp.dig_P8)) >> 13;
	P = (uint32_t)((int32_t)P + ((var1 + var2 + BMP280->Comp.dig_P7) >> 4));
	BMP280->PressUint32 = P;
}

//Перевод давления из Па в мм. рт. ст.
void BMP280_GetmmHg (BMP280_TypeDef *BMP280)
{
	if (BMP280->PressUint32 != 0){
		BMP280->mmHgUint32 = BMP280->PressUint32 / (101325/760);
	}

	if (BMP280->PressDouble != 0){
		BMP280->mmHgDouble = BMP280->PressDouble / (101325.0/760.0);
	}
}
