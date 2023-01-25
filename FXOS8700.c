#include "stm32l0xx_hal.h"
#include "FXOS8700.h"
#include "math.h"
#include "FXOS8700_registers.h"



#define InputLPFKoeff 100//default 100
#define AccCalThreshold 5//default 5
#define MagCalThreshold 5//default 5
#define NumberOfCalibrationPoints 6//default 6
#define CalibrationDistanceThreshold 200//default 200
#define DataStableCyclesThreshold 25//default 25
#define VariationDepth 3//default 3


#define M_PI ((float)3.141592653589793)
#define M_PI12 (M_PI/12.F)
#define M_PI6 (M_PI/6.F)
#define M_PI2 (M_PI/2.F)
/* square root of 3 */
#define SQRT3 ((float)1.732050807569)
#define	HALF_RANGE	32767
#define FULL_RANGE	65535
#define RADIANS_TO_DEGREES	57.295779513082320876798154814105

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;


SRAWDATA rawAccel;
SRAWDATA rawMag;
SRAWDATA rawAccelPrevious;
SRAWDATA rawMagPrevious;
SRAWDATA accelFiltered;
SRAWDATA magFiltered;

SRAWDATA accelOffset;
SRAWDATA magOffset;
SRAWDATA accelScale;
SRAWDATA magScale;
SRAWDATA magScale;
SRAWDATA accelOffsetResult;
SRAWDATA magOffsetResult;
SRAWDATA accelScaleResult;
SRAWDATA magScaleResult;

SRAWDATA accelOffsetCurrent;
SRAWDATA magOffsetCurrent;
SRAWDATA accelScaleCurrent;
SRAWDATA magScaleCurrent;

int8_t reportCounter = 0;
int32_t radius;
int32_t radiusResult;
int32_t radiusCurrent;
int32_t radiusMag;
int32_t radiusResultMag;
int32_t radiusCurrentMag;
int32_t calibrationError = 0xFFFFFFFF;
float_t floatBuffer;
float_t mismatchArray[NumberOfCalibrationPoints];
float_t mismatchArrayMag[NumberOfCalibrationPoints];

float_t distancesArray[NumberOfCalibrationPoints];
float_t meanDistance;
float_t	deCenter = 0;
float_t	deCenterMag = 0;
float_t meanMismatch = 0;
float_t meanMismatchMag = 0;
uint32_t multiplier = 2;
	
float_t floatCalError = 0xffffffffffffffff;
float_t floatCenterCalError = 0xffffffffffffffff;
float_t floatCalErrorPrevious;
float_t floatCenterCalErrorPrevious;
float_t floatMagCalError = 0xffffffffffffffff;
float_t floatMagCenterCalError = 0xffffffffffffffff;
float_t floatMagCalErrorPrevious;
float_t floatMagCenterCalErrorPrevious;
uint32_t _Buffer;

SRAWDATA rawAccelCalibrationPoints[NumberOfCalibrationPoints];
SRAWDATA rawMagCalibrationPoints[NumberOfCalibrationPoints];


void FXOS8700_debugMessageAsHEX(int8_t message)
{
	char msg[64];
	snprintf(msg, sizeof(msg), "0x%02X", message);	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void FXOS8700_debugMessageBetterMagMismatch()
{
	char msg[64];
	snprintf(msg, sizeof(msg), "a better mismatch detected: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatBuffer, (uint16_t)((floatBuffer - (uint32_t)floatBuffer) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "offset: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", magOffsetResult.x);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", magOffsetResult.y);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", magOffsetResult.z);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "scale: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						
	snprintf(msg, sizeof(msg), "%+04i", magScaleResult.x);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", magScaleResult.y);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", magScaleResult.z);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "; ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

	snprintf(msg, sizeof(msg), "radius: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", radiusResultMag);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "   multiplier: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "\r\n");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void FXOS8700_debugMessageBetterAccelMismatch()
{
	char msg[64];
	snprintf(msg, sizeof(msg), "a better mismatch detected: ");	
	snprintf(msg, sizeof(msg), "a better mismatch detected: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatBuffer, (uint16_t)((floatBuffer - (uint32_t)floatBuffer) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "offset: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.x);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.y);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.z);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "scale: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						
	snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.x);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.y);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), ", ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.z);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "; ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

	snprintf(msg, sizeof(msg), "radius: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", radiusResult);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "   multiplier: ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "\r\n");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void FXOS8700_debugMessageCalPointNmb(uint8_t calpointNmb)
	
{
	char msg[64];
	snprintf(msg, sizeof(msg), "\r\n");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "a new point# ");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "%+04i", calpointNmb + 1);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), " added");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
					
	snprintf(msg, sizeof(msg), "\r\n");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void FXOS8700_debugMessageString(char message[])
{
	char msg[64];
	sprintf(msg, message);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

float Arctan(float x) {
	int sta = 0, sp = 0;
	float x2, a;
	/* check up the sign change */
	if (x < 0.F) {x = -x; sta |= 1; }
	/* check up the invertation */
	if (x > 1.F) {x = 1.F / x; sta |= 2; }
	/* process shrinking the domain until x<PI/12 */
	while (x > M_PI12) {
		sp++; a = x + SQRT3; a = 1.F / a; x *= SQRT3; x -= 1.F; x *= a;
	}
	/* calculation core */
	x2 = x*x; a = x2 + 1.4087812F; a = 0.55913709F / a; a += 0.60310579F;
	a -= 0.05160454F*x2; a *= x;
	/* process until sp=0 */
	while (sp > 0) {a += M_PI6; sp--; }
	/* invertation took place */
	if (sta & 2) a = M_PI2 - a;
	/* sign change took place */
	if (sta & 1) a = -a;
	return (a);
}

///a function to check if the FXOS8700 chip is online
///
///
uint8_t FXOS8700_checkResponse()
{
	if (HAL_I2C_IsDeviceReady(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, 1, HAL_MAX_DELAY) == HAL_OK)
	{
		return FXOS8700_OK;
	}
	else
	{
		return FXOS8700_COMMUNICATION_ERROR;
	}
}





///a function to read data from FXOS8700
///
///reads acc and mag data subsequently. The corresponding subsequent mode must be set first.
///This may be done by calling the FXOS8700_init() function
uint8_t FXOS8700_readAccelAndMag(SRAWDATA *pAccelData, SRAWDATA *pMagnData)
{
	uint8_t result = FXOS8700_COMMUNICATION_ERROR;
	uint8_t buf[2];
	uint8_t Buffer[FXOS8700CQ_READ_LEN];
	
	Buffer[0] = FXOS8700CQ_STATUS;
	if (HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, Buffer[0], I2C_MEMADD_SIZE_8BIT, (uint8_t *)&Buffer[0], FXOS8700CQ_READ_LEN, 1000) == HAL_OK)
	{
		result = FXOS8700_OK;
	}
	

	
	
	// copy the accelerometer byte data into 16 bit words
	pAccelData->x = (Buffer[1] << 8) | Buffer[2];
	pAccelData->y = (Buffer[3] << 8) | Buffer[4];
	pAccelData->z = (Buffer[5] << 8) | Buffer[6];
	// copy the magnetometer byte data into 16 bit words
	pMagnData->x = (Buffer[7] << 8) | Buffer[8];
	pMagnData->y = (Buffer[9] << 8) | Buffer[10];
	pMagnData->z = (Buffer[11] << 8) | Buffer[12];
	
	return result;
	
	
}

int32_t FXOS8700_getMagneticHeading(CALIBRATIONRESULT calibData, uint8_t *ERROR_CODE)
{
	*ERROR_CODE = FXOS8700_OK;
	for (int16_t i = 0; i < InputLPFKoeff; i++)
	{
		if (FXOS8700_readAccelAndMag(&rawAccel, &rawMag) == FXOS8700_OK)
		{
			if (rawAccel.x >= HALF_RANGE)
			{
				rawAccel.x = rawAccel.x - FULL_RANGE;
			}
			if (rawAccel.y >= HALF_RANGE)
			{
				rawAccel.y = rawAccel.y - FULL_RANGE;
			}
			if (rawAccel.z >= HALF_RANGE)
			{
				rawAccel.z = rawAccel.z - FULL_RANGE;
			}
			if (rawMag.x >= HALF_RANGE)
			{
				rawMag.x = rawMag.x - FULL_RANGE;
			}
			if (rawMag.y >= HALF_RANGE)
			{
				rawMag.y = rawMag.y - FULL_RANGE;
			}
			if (rawMag.z >= HALF_RANGE)
			{
				rawMag.z = rawMag.z - FULL_RANGE;
			}
			rawAccel.x = rawAccel.x * InputLPFKoeff;
			accelFiltered.x = (accelFiltered.x * (InputLPFKoeff - 1) / InputLPFKoeff + rawAccel.x * 1 / InputLPFKoeff);
			rawAccel.y = rawAccel.y * InputLPFKoeff;
			accelFiltered.y = (accelFiltered.y * (InputLPFKoeff - 1) / InputLPFKoeff + rawAccel.y * 1 / InputLPFKoeff);
			rawAccel.z = rawAccel.z * InputLPFKoeff;
			accelFiltered.z = (accelFiltered.z * (InputLPFKoeff - 1)  / InputLPFKoeff + rawAccel.z * 1 / InputLPFKoeff);
			
			rawMag.x = rawMag.x * InputLPFKoeff;
			magFiltered.x = (magFiltered.x * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.x * 1 / InputLPFKoeff);
			rawMag.y = rawMag.y * InputLPFKoeff;
			magFiltered.y = (magFiltered.y * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.y * 1 / InputLPFKoeff);
			rawMag.z = rawMag.z * InputLPFKoeff;
			magFiltered.z = (magFiltered.z * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.z * 1 / InputLPFKoeff);
		
		}
		else
		{
			*ERROR_CODE = FXOS8700_COMMUNICATION_ERROR;
#ifdef FXOS8700_SERIAL_DEBUG
			FXOS8700_debugMessageString("ERROR reading raw values\r\n");
#endif

		}
		

	}
	int32_t buf;
	float_t buf1;
	float_t buf2;
	float_t buf3;
	
	float_t buf11;
	float_t buf12;
	float_t buf13;
	
	float_t accPitch;
	float_t accRoll;
	float_t accYaw;
	
	float_t magPitch;
	float_t magRoll;
	float_t magYaw;
	
	float_t xHeading;
	float_t yHeading;
	
	float_t magHeading;
	float_t magHeadingNonCompensated;
	
	buf1 = ((float_t)accelFiltered.x / (float_t)InputLPFKoeff  - (float_t)calibData.aox / (float_t)InputLPFKoeff);
	buf1 = buf1*calibData.asx;
	
	buf2 = ((float_t)accelFiltered.y / (float_t)InputLPFKoeff  - (float_t)calibData.aoy / (float_t)InputLPFKoeff);
	buf2 = buf2*calibData.asy;
	
	buf3 = ((float_t)accelFiltered.z / (float_t)InputLPFKoeff  - (float_t)calibData.aoz / (float_t)InputLPFKoeff);
	buf3 = buf3*calibData.asz;

	accPitch = asinf((0 - buf1) / sqrtf(buf1*buf1 + buf2*buf2 + buf3*buf3));
	accPitch = accPitch * RADIANS_TO_DEGREES;
	
	accRoll = asin((buf2 / sqrtf(buf1*buf1 + buf2*buf2 + buf3*buf3)) / cos(accPitch / RADIANS_TO_DEGREES));
	accRoll = accRoll * RADIANS_TO_DEGREES;
	
	accYaw = atan2f(buf1, buf2);
	accYaw = accYaw * RADIANS_TO_DEGREES;
	
	buf11 = ((float_t)magFiltered.x / (float_t)InputLPFKoeff  - (float_t)calibData.mox / (float_t)InputLPFKoeff);
	buf11 = buf11*calibData.msx;
	
	buf12 = ((float_t)magFiltered.y / (float_t)InputLPFKoeff  - (float_t)calibData.moy / (float_t)InputLPFKoeff);
	buf12 = buf12*calibData.msy;
	
	buf13 = ((float_t)magFiltered.z / (float_t)InputLPFKoeff  - (float_t)calibData.moz / (float_t)InputLPFKoeff);
	buf13 = buf13*calibData.msz;
	
	
	magPitch = atan2f(buf11, buf13);
	magPitch = magPitch * RADIANS_TO_DEGREES;
	
	magRoll = atan2f(buf12, buf13);
	magRoll = magRoll * RADIANS_TO_DEGREES;
	
	magYaw = atan2f(buf11, buf12);
	magYaw = magYaw * RADIANS_TO_DEGREES;
	
	
	xHeading = buf11*cos(accPitch / RADIANS_TO_DEGREES) + buf13*sin(accPitch / RADIANS_TO_DEGREES);
	yHeading = buf11*sin(accRoll / RADIANS_TO_DEGREES)*sin(accPitch / RADIANS_TO_DEGREES) + buf12*cos(accRoll / RADIANS_TO_DEGREES) - buf13*sin(accRoll / RADIANS_TO_DEGREES)*cos(accPitch / RADIANS_TO_DEGREES);
	
	magHeadingNonCompensated = magYaw; 
	magHeading = atan2f(yHeading, xHeading) * RADIANS_TO_DEGREES;
	
	/*
	char msg[64];
	sprintf(msg, "accPitch:  %d.%03d", (uint32_t)accPitch, (uint16_t)((accPitch - (uint32_t)accPitch) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " accRoll:  %d.%03d", (uint32_t)accRoll, (uint16_t)((accRoll - (uint32_t)accRoll) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " accYaw:  %d.%03d", (uint32_t)accYaw, (uint16_t)((accYaw - (uint32_t)accYaw) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " magPitch:  %d.%03d", (uint32_t)magPitch, (uint16_t)((magPitch - (uint32_t)magPitch) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " magRoll:  %d.%03d", (uint32_t)magRoll, (uint16_t)((magRoll - (uint32_t)magRoll) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " magYaw:  %d.%03d", (uint32_t)magYaw, (uint16_t)((magYaw - (uint32_t)magYaw) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " magHNC:  %d.%03d", (uint32_t)magHeadingNonCompensated, (uint16_t)((magHeadingNonCompensated - (uint32_t)magHeadingNonCompensated) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	sprintf(msg, " magH:  %d.%03d", (uint32_t)magHeading, (uint16_t)((magHeading - (uint32_t)magHeading) * 1000.));
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	snprintf(msg, sizeof(msg), "\r\n");	
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	*/
	buf = (int32_t)(magHeading * 1000);
	return buf;
}

uint8_t FXOS8700_test()
{
	uint8_t result = FXOS8700_OK;
	uint8_t buf[2];
	uint8_t databyte[8];
	//char msg[64];
	
	buf[0] = FXOS8700CQ_CTRL_REG1;
	buf[1] = 0x00;
	if (!(HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) == HAL_OK))
	{
		result = FXOS8700_COMMUNICATION_ERROR;
	}
	
	buf[0] = FXOS8700CQ_M_CTRL_REG1;
	buf[1] = 0x1F;
	if (!(HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) ==HAL_OK))
	{
		result = FXOS8700_COMMUNICATION_ERROR;
	}
	
	//HAL_Delay(10);
	buf[0] = FXOS8700CQ_M_CTRL_REG2;
	buf[1] = 0x20;
	if (!(HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) == HAL_OK))
	{
		result = FXOS8700_COMMUNICATION_ERROR;
	}
	
	//HAL_Delay(10);
	buf[0] = FXOS8700CQ_XYZ_DATA_CFG;
	buf[1] = 0x01;
	if (!(HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) == HAL_OK))
	{
		result = FXOS8700_COMMUNICATION_ERROR;
	}

	
	//HAL_Delay(10);
	
	buf[0] = FXOS8700CQ_STATUS;
	
	//HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 1, 1000);
	if(HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf[0], I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buf[0], 1, 1000) == HAL_OK)
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageAsHEX(buf[0]);
#endif//FXOS8700_SERIAL_DEBUG
	}
	else
	{
		buf[0] = 0x03;
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageAsHEX(buf[0]);
#endif // FXOS8700_SERIAL_DEBUG		
		result = FXOS8700_COMMUNICATION_ERROR;
	}
	
	return result;
}


///a function to initialize FXOS8700 chip
///
///sets the following: 200-Hz hybrid mode. Both accelerometer and magnetometer data
///are provided at the 200 - Hz rate
uint8_t FXOS8700_init()
{
	uint8_t result = FXOS8700_OK;
	uint8_t buf[2];
	uint8_t databyte[8];
	//HAL_Delay(1000);
	buf[0] = FXOS8700CQ_WHOAMI;
	if (HAL_I2C_Mem_Read(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf[0], I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buf[0], 1, 1000) == HAL_OK)
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageAsHEX(buf[0]);
#endif //FXOS8700_SERIAL_DEBUG
		if (buf[0] == FXOS8700CQ_WHOAMI_VAL)
		{
			buf[0] = FXOS8700CQ_CTRL_REG1;
			// write 0000 0000 = 0x00 to accelerometer control register 1 to place FXOS8700 into
			// standby
			// [7-1] = 0000 000
			// [0]: active=0
			buf[1] = 0x00;
			
			buf[1] |= FXOS8700CQ_CTRL_REG1_INACTIVE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_NORMAL_READ_MODE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_NORMAL_NOISE_MODE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_0;
			buf[1] |= FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_50Hz;
		
			if (HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) != HAL_OK) 
			{
				result = FXOS8700_COMMUNICATION_ERROR;
			}
			
			buf[0] = FXOS8700CQ_M_CTRL_REG1;
			// write 0001 1111 = 0x1F to magnetometer control register 1
			// [7]: m_acal=0: auto calibration disabled
			// [6]: m_rst=0: no one-shot magnetic reset
			// [5]: m_ost=0: no one-shot magnetic measurement
			// [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
			// [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
			//buf[1] = 0x1F;
			buf[1] = 0x00;
			
			buf[1] |= FXOS8700CQ_M_CTRL_REG1_HYBRID_MODE;
			buf[1] |= FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_7;
			buf[1] |= FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_MEASURE_OFF;
			buf[1] |= FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_RST_OFF;
			buf[1] |= FXOS8700CQ_M_CTRL_REG1_AUTO_CALIBRATION_OFF;
			
			if (HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) != HAL_OK)
			{
				result = FXOS8700_COMMUNICATION_ERROR;
			}
			buf[0] = FXOS8700CQ_M_CTRL_REG2;
			// write 0010 0000 = 0x20 to magnetometer control register 2
			// [7]: reserved
			// [6]: reserved
			// [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
			// accelerometer registers
			// [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
			// [3]: m_maxmin_dis_ths=0
			// [2]: m_maxmin_rst=0
			// [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
			//buf[1] = 0x20;
			buf[1] = 0x00;
			
			buf[1] |= FXOS8700CQ_M_CTRL_REG2_MAG_RST_EVERY_1_CYCLE;
			buf[1] |= FXOS8700CQ_M_CTRL_REG2_MAG_MAXIM_RST_OFF;
			buf[1] |= FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_THD_OFF;
			buf[1] |= FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_OFF;
			buf[1] |= FXOS8700CQ_M_CTRL_REG2_MAG_AUTOINCREMENT_ON;
			
			
			if (HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) != HAL_OK)
			{
				result = FXOS8700_COMMUNICATION_ERROR;
			}
				
			buf[0] = FXOS8700CQ_XYZ_DATA_CFG;
			// write 0000 0001= 0x01 to XYZ_DATA_CFG register
			// [7]: reserved
			// [6]: reserved
			// [5]: reserved
			// [4]: hpf_out=0
			// [3]: reserved
			// [2]: reserved
			// [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB
			//buf[1] = 0x01;
			buf[1] = 0x00;
			
			buf[1] |= FXOS8700CQ_XYZ_DATA_CFG_RANGE_0_488mg_per_LSB;
			buf[1] |= FXOS8700CQ_XYZ_DATA_CFG_HPF_OFF;
			
			if (HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) != HAL_OK)
			{
				result = FXOS8700_COMMUNICATION_ERROR;
			}
				
			buf[0] = FXOS8700CQ_CTRL_REG1;
			// write 0000 1101b = 0x0D to accelerometer control register 1
			// [7-6]: aslp_rate=00
			// [5-3]: dr=001=1 for 200Hz data rate (when in hybrid mode)
			// [2]: lnoise=1 for low noise mode
			// [1]: f_read=0 for normal 16 bit reads
			// [0]: active=1 to take the part out of standby and enable sampling
			//buf[1] = 0x0D;
			buf[1] = 0x00;
			
			buf[1] |= FXOS8700CQ_CTRL_REG1_ACTIVE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_NORMAL_READ_MODE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_REDUCED_NOISE_MODE;
			buf[1] |= FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_1;
			buf[1] |= FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_50Hz;
			
			
			if (HAL_I2C_Master_Transmit(&hi2c2, FXOS8700CQ_SLAVE_ADDR << 1, buf, 2, 1000) != HAL_OK)
			{
				result = FXOS8700_COMMUNICATION_ERROR;
			}
		}
		else
		{
			result = FXOS8700_COMMUNICATION_ERROR;
		}
	}
	else
	{
		result = FXOS8700_COMMUNICATION_ERROR;
	}

	return result;
}

///a function to calculate calibration error
///
///this function calculates calibration error as a sum mismatch in approximation ellipsoid formula
float_t mismatch(SRAWDATA point, SRAWDATA offset, SRAWDATA scale, uint32_t radius)
{
	float_t buf;
	float_t buf1;
	float_t buf2;
	float_t buf3;
	//buf = ((float_t)point.x  - (float_t)offset.x)*((float_t)point.x - (float_t)offset.x)*((float_t)scale.x / 1000000) + ((float_t)point.y - (float_t)offset.y)*((float_t)point.y - (float_t)offset.y)*((float_t)scale.y / 1000000) + ((float_t)point.z - (float_t)offset.z)*((float_t)point.z  - (float_t)offset.z)*((float_t)scale.z / 1000000) - ((float_t)radius)*((float_t)radius)/1000;
	
	buf1 = ((float_t)point.x / (float_t)InputLPFKoeff  - (float_t)offset.x / (float_t)InputLPFKoeff)*((float_t)point.x / (float_t)InputLPFKoeff - (float_t)offset.x / (float_t)InputLPFKoeff);
	buf1 = buf1 / 1000;
	buf1 = buf1*scale.x;
	
	buf2 = ((float_t)point.y / (float_t)InputLPFKoeff  - (float_t)offset.y / (float_t)InputLPFKoeff)*((float_t)point.y / (float_t)InputLPFKoeff - (float_t)offset.y / (float_t)InputLPFKoeff);
	buf2 = buf2 / 1000;
	buf2 = buf2*scale.y;
	
	
	buf3 = ((float_t)point.z / (float_t)InputLPFKoeff  - (float_t)offset.z / (float_t)InputLPFKoeff)*((float_t)point.z / (float_t)InputLPFKoeff - (float_t)offset.z / (float_t)InputLPFKoeff);
	buf3 = buf3 / 1000;
	buf3 = buf3*scale.z;
	
	buf = ((float_t)radius / (float_t)InputLPFKoeff)*((float_t)radius / (float_t)InputLPFKoeff);
	if ((buf1 + buf2 + buf3) > buf)
	{
		buf = buf1 + buf2 + buf3 - buf;
	}
	else
	{
		
		buf = buf - buf1 - buf2 - buf3;
	}
	return buf;
}

///a function that searches for the best ellipsoid - approximated solution for the magnetic point cloud
void FXOS8700_gradientSearchMag(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul)
{

	
	for (int16_t a = 0; a < rad1; a++)
	{
		for (int16_t b = 0; b < rad2; b++)
		{
			for (int16_t c = 0; c < rad3; c++)
			{
				for (int16_t d = 0; d < rad4; d++)
				{
					for (int16_t e = 0; e < rad5; e++)
					{
						for (int16_t f = 0; f < rad6; f++)
						{
							for (int16_t g = 0; g < rad7; g++)
							{
								if (rad1 > 1)
								{
									magOffset.x = magOffsetCurrent.x + (a - (rad1 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad2 > 1)
								{
									magOffset.y = magOffsetCurrent.y + (b - (rad2 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad3 > 1)
								{
									magOffset.z = magOffsetCurrent.z + (c - (rad3 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad4 > 1)
								{
									magScale.x = magScaleCurrent.x + (d - (rad4 - 1) / 2)*mul * 10;
								}
								if (rad5 > 1)
								{
									magScale.y = magScaleCurrent.y + (e - (rad5 - 1) / 2)*mul * 10;
								}
								if (rad6 > 1)
								{
									magScale.z = magScaleCurrent.z + (f - (rad6 - 1) / 2)*mul * 10;
								}
													
								if (rad7 > 1)
								{
									radiusMag = radiusCurrentMag + (g - (rad7 - 1) / 2)*InputLPFKoeff*mul;
								}
													
								//snprintf(msg, sizeof(msg), "radiusMag: ");	
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//snprintf(msg, sizeof(msg), "%+04i", radiusMag);
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//snprintf(msg, sizeof(msg), "\r\n");	
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//HAL_Delay(100);
													
													
								_Buffer = 0;
								floatBuffer = 0;
						
								
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									mismatchArrayMag[o] = mismatch(rawMagCalibrationPoints[o], magOffset, magScale, radiusMag);
									floatBuffer = floatBuffer + mismatchArrayMag[o];
								}
								deCenterMag = 0;
								meanMismatchMag = 0;
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									meanMismatchMag = meanMismatchMag + mismatchArrayMag[o] / NumberOfCalibrationPoints;
								}
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									if (meanMismatchMag > mismatchArrayMag[o])
									{
										deCenterMag = deCenterMag + (meanMismatchMag - mismatchArrayMag[o]);
									}
									else
									{
										deCenterMag = deCenterMag + (mismatchArrayMag[o] - meanMismatchMag);
									}
								}
													
								//floatBuffer = floatBuffer + 1000*deCenter;
								//floatBuffer = deCenter;
								
								/*
								snprintf(msg, sizeof(msg), "current mismatch is: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatBuffer, (uint16_t)((floatBuffer - (uint32_t)floatBuffer) * 1000.));
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "offset: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.x);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.y);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.z);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "scale: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						
								snprintf(msg, sizeof(msg), "%+04i", accelScale.x);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelScale.y);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelScale.z);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "; ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

								snprintf(msg, sizeof(msg), "radius: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", radius);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "   multiplier: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "\r\n");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(100);
								*/
													
								if(floatBuffer < floatMagCalError)
								{
#ifdef FXOS8700_SERIAL_DEBUG
									FXOS8700_debugMessageBetterMagMismatch();
#endif // FXOS8700_SERIAL_DEBUG
									HAL_Delay(100);
													
									magOffsetResult = magOffset;
									magScaleResult = magScale;
									radiusResultMag = radiusMag;
									//calibrationError = _Buffer;
									floatMagCalError = floatBuffer;
									//HAL_Delay(100);
								}
													
													
													
													
													
													
							}
						}
					}
				}
			}
		}
	}
	magOffsetCurrent =  magOffsetResult;
	magScaleCurrent = magScaleResult;
	radiusCurrentMag = radiusResultMag;
}

///a function that searches for the best ellipsoid - approximated solution for the acceleration point cloud
void FXOS8700_gradientSearch(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul)
{

	//char msg[64];
	for (int16_t a = 0; a < rad1; a++)
	{
		for (int16_t b = 0; b < rad2; b++)
		{
			for (int16_t c = 0; c < rad3; c++)
			{
				for (int16_t d = 0; d < rad4; d++)
				{
					for (int16_t e = 0; e < rad5; e++)
					{
						for (int16_t f = 0; f < rad6; f++)
						{
							for (int16_t g = 0; g < rad7; g++)
							{
								if (rad1 > 1)
								{
									accelOffset.x = accelOffsetCurrent.x + (a - (rad1 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad2 > 1)
								{
									accelOffset.y = accelOffsetCurrent.y + (b - (rad2 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad3 > 1)
								{
									accelOffset.z = accelOffsetCurrent.z + (c - (rad3 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad4 > 1)
								{
									accelScale.x = accelScaleCurrent.x + (d - (rad4 - 1) / 2)*mul * 10;
								}
								if (rad5 > 1)
								{
									accelScale.y = accelScaleCurrent.y + (e - (rad5 - 1) / 2)*mul * 10;
								}
								if (rad6 > 1)
								{
									accelScale.z = accelScaleCurrent.z + (f - (rad6 - 1) / 2)*mul * 10;
								}
													
								if (rad7 > 1)
								{
									radius = radiusCurrent + (g - (rad7 - 1) / 2)*InputLPFKoeff*mul;
								}
													
								//snprintf(msg, sizeof(msg), "radius: ");	
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//snprintf(msg, sizeof(msg), "%+04i", radius);
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//snprintf(msg, sizeof(msg), "\r\n");	
								//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								//HAL_Delay(100);
													
													
								_Buffer = 0;
								floatBuffer = 0;
						
								
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									mismatchArray[o] = mismatch(rawAccelCalibrationPoints[o], accelOffset, accelScale, radius);
									floatBuffer = floatBuffer + mismatchArray[o];
								}
								deCenter = 0;
								meanMismatch = 0;
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									meanMismatch = meanMismatch + mismatchArray[o] / NumberOfCalibrationPoints;
								}
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									if (meanMismatch > mismatchArray[o])
									{
										deCenter = deCenter + (meanMismatch - mismatchArray[o]);
									}
									else
									{
										deCenter = deCenter + (mismatchArray[o] - meanMismatch);
									}
								}
													
								//floatBuffer = floatBuffer + 1000*deCenter;
								//floatBuffer = deCenter;
								
								/*
								snprintf(msg, sizeof(msg), "current mismatch is: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatBuffer, (uint16_t)((floatBuffer - (uint32_t)floatBuffer) * 1000.));
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "offset: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.x);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.y);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelOffset.z);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "scale: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						
								snprintf(msg, sizeof(msg), "%+04i", accelScale.x);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelScale.y);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), ", ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", accelScale.z);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "; ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

								snprintf(msg, sizeof(msg), "radius: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", radius);
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "   multiplier: ");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								snprintf(msg, sizeof(msg), "\r\n");	
								HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
								HAL_Delay(100);
								*/
													
								if(floatBuffer < floatCalError)
								{
#ifdef FXOS8700_SERIAL_DEBUG
									FXOS8700_debugMessageBetterAccelMismatch();
									HAL_Delay(100);
#endif//FXOS8700_SERIAL_DEBUG
									accelOffsetResult = accelOffset;
									accelScaleResult = accelScale;
									radiusResult = radius;
									//calibrationError = _Buffer;
									floatCalError = floatBuffer;
									//HAL_Delay(100);
								}
													
													
													
													
													
													
							}
						}
					}
				}
			}
		}
	}
	accelOffsetCurrent =  accelOffsetResult;
	accelScaleCurrent = accelScaleResult;
	radiusCurrent = radiusResult;
}

///a function that searches for the best ellipsoid - approximated solution for the center of the magnetic pointcloud
void FXOS8700_gradientCenterSearchMag(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul)
{

	//char msg[64];
	for (int16_t a = 0; a < rad1; a++)
	{
		for (int16_t b = 0; b < rad2; b++)
		{
			for (int16_t c = 0; c < rad3; c++)
			{
				for (int16_t d = 0; d < rad4; d++)
				{
					for (int16_t e = 0; e < rad5; e++)
					{
						for (int16_t f = 0; f < rad6; f++)
						{
							for (int16_t g = 0; g < rad7; g++)
							{
								if (rad1 > 1)
								{
									magOffset.x = magOffsetCurrent.x + (a - (rad1 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad2 > 1)
								{
									magOffset.y = magOffsetCurrent.y + (b - (rad2 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad3 > 1)
								{
									magOffset.z = magOffsetCurrent.z + (c - (rad3 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad4 > 1)
								{
									magScale.x = magScaleCurrent.x + (d - (rad4 - 1) / 2)*mul * 10;
								}
								if (rad5 > 1)
								{
									magScale.y = magScaleCurrent.y + (e - (rad5 - 1) / 2)*mul * 10;
								}
								if (rad6 > 1)
								{
									magScale.z = magScaleCurrent.z + (f - (rad6 - 1) / 2)*mul * 10;
								}
													
								if (rad7 > 1)
								{
									radiusMag = radiusCurrentMag + (g - (rad7 - 1) / 2)*InputLPFKoeff*mul;
								}
													
					
													
								_Buffer = 0;
								floatBuffer = 0;
						
								
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									distancesArray[o] = (rawMagCalibrationPoints[o].x - (float_t)magOffset.x)*(rawMagCalibrationPoints[o].x - (float_t)magOffset.x); 
									distancesArray[o] = distancesArray[o] + (rawMagCalibrationPoints[o].y - (float_t)magOffset.y)*(rawMagCalibrationPoints[o].y - (float_t)magOffset.y);
									distancesArray[o] = distancesArray[o] + (rawMagCalibrationPoints[o].z - (float_t)magOffset.z)*(rawMagCalibrationPoints[o].z - (float_t)magOffset.z);
									distancesArray[o] = sqrtf(distancesArray[o]);
								}
								deCenterMag = 0;
								meanDistance = 0;
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									meanDistance = meanDistance + distancesArray[o] / NumberOfCalibrationPoints;
								}
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									if (meanDistance > distancesArray[o])
									{
										deCenterMag = deCenterMag + (meanDistance - distancesArray[o]);
									}
									else
									{
										deCenterMag = deCenterMag + (distancesArray[o] - meanDistance);
									}
									
									
									
								}
													
								
								floatBuffer = deCenterMag;
								
								
													
								if (floatBuffer < floatMagCenterCalError)
								{
													
									magOffsetResult = magOffset;
									magScaleResult = magScale;
									radiusResultMag = radiusMag;
									//calibrationError = _Buffer;
									floatMagCenterCalError = floatBuffer;
									//HAL_Delay(100);
									
									reportCounter++;
									if (reportCounter >= 100)
									{
										reportCounter = 0;
#ifdef FXOS8700_SERIAL_DEBUG
										FXOS8700_debugMessageBetterMagMismatch();
#endif//FXOS8700_SERIAL_DEBUG
										HAL_Delay(100);
									}
									
									
								}
													
													
													
													
													
													
							}
						}
					}
				}
			}
		}
	}
	magOffsetCurrent =  magOffsetResult;
	magScaleCurrent = magScaleResult;
	radiusCurrentMag = radiusResultMag;
}	


///a function that searches for the best ellipsoid - approximated solution for the center of the acceleration pointcloud
void FXOS8700_gradientCenterSearch(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul)
{

	char msg[64];
	for (int16_t a = 0; a < rad1; a++)
	{
		for (int16_t b = 0; b < rad2; b++)
		{
			for (int16_t c = 0; c < rad3; c++)
			{
				for (int16_t d = 0; d < rad4; d++)
				{
					for (int16_t e = 0; e < rad5; e++)
					{
						for (int16_t f = 0; f < rad6; f++)
						{
							for (int16_t g = 0; g < rad7; g++)
							{
								if (rad1 > 1)
								{
									accelOffset.x = accelOffsetCurrent.x + (a - (rad1 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad2 > 1)
								{
									accelOffset.y = accelOffsetCurrent.y + (b - (rad2 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad3 > 1)
								{
									accelOffset.z = accelOffsetCurrent.z + (c - (rad3 - 1) / 2)*InputLPFKoeff*mul;
								}
								if (rad4 > 1)
								{
									accelScale.x = accelScaleCurrent.x + (d - (rad4 - 1) / 2)*mul * 10;
								}
								if (rad5 > 1)
								{
									accelScale.y = accelScaleCurrent.y + (e - (rad5 - 1) / 2)*mul * 10;
								}
								if (rad6 > 1)
								{
									accelScale.z = accelScaleCurrent.z + (f - (rad6 - 1) / 2)*mul * 10;
								}
													
								if (rad7 > 1)
								{
									radius = radiusCurrent + (g - (rad7 - 1) / 2)*InputLPFKoeff*mul;
								}
													
					
													
								_Buffer = 0;
								floatBuffer = 0;
						
								
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									distancesArray[o] = (rawAccelCalibrationPoints[o].x - (float_t)accelOffset.x)*(rawAccelCalibrationPoints[o].x - (float_t)accelOffset.x); 
									distancesArray[o] = distancesArray[o] + (rawAccelCalibrationPoints[o].y - (float_t)accelOffset.y)*(rawAccelCalibrationPoints[o].y - (float_t)accelOffset.y);
									distancesArray[o] = distancesArray[o] + (rawAccelCalibrationPoints[o].z - (float_t)accelOffset.z)*(rawAccelCalibrationPoints[o].z - (float_t)accelOffset.z);
									distancesArray[o] = sqrtf(distancesArray[o]);
								}
								deCenter = 0;
								meanDistance = 0;
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									meanDistance = meanDistance + distancesArray[o] / NumberOfCalibrationPoints;
								}
													
								for (uint16_t o = 0; o < NumberOfCalibrationPoints; o++)
								{
									if (meanDistance > distancesArray[o])
									{
										deCenter = deCenter + (meanDistance - distancesArray[o]);
									}
									else
									{
										deCenter = deCenter + (distancesArray[o] - meanDistance);
									}
									
									
									
								}
													
								
								floatBuffer = deCenter;
								
								
													
								if (floatBuffer < floatCenterCalError)
								{
													
									accelOffsetResult = accelOffset;
									accelScaleResult = accelScale;
									radiusResult = radius;
									//calibrationError = _Buffer;
									floatCenterCalError = floatBuffer;
									//HAL_Delay(100);
								}
													
													
													
													
													
													
							}
						}
					}
				}
			}
		}
	}
	accelOffsetCurrent =  accelOffsetResult;
	accelScaleCurrent = accelScaleResult;
	radiusCurrent = radiusResult;
}	
	


///a function that collects calibration pointcloud and calculates cefficients of an approximation ellipsoid
void FXOS8700_calibrate(CALIBRATIONRESULT *output, int8_t accurateMode)
{
	accelFiltered.x = 0;
	accelFiltered.y = 0;
	accelFiltered.z = 0;
		
	magFiltered.x = 0;
	magFiltered.y = 0;
	magFiltered.z = 0;
	uint8_t calPointsCounter = 0;
	uint8_t stableDataCounter = 0;
	
	
#ifdef FXOS8700_SERIAL_DEBUG
	FXOS8700_debugMessageString("Checking if FXOS8700 is online....\r\n");
#endif


	
	if(FXOS8700_checkResponse() == FXOS8700_OK)
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageString("FXOS8700 is ONLINE!(\r\n");
#endif
		
	}
	else
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageString("FXOS8700 is OFFLINE(\r\n");
#endif

	}
#ifdef FXOS8700_SERIAL_DEBUG
	FXOS8700_debugMessageString("Initializing FXOS8700 for continuous conversion....\r\n");
#endif

	
	if(FXOS8700_init() == FXOS8700_OK)
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageString("FXOS8700 has recieved configuration.\r\n");
#endif
	}
	else
	{
#ifdef FXOS8700_SERIAL_DEBUG
		FXOS8700_debugMessageString("An error occured while configuring an FXOS8700 chip.\r\n");
#endif
	}
	while (calPointsCounter <= NumberOfCalibrationPoints)
	{
		for (int16_t i = 0; i < InputLPFKoeff; i++)
		{
			if (FXOS8700_readAccelAndMag(&rawAccel, &rawMag) == FXOS8700_OK)
			{
				if (rawAccel.x >= HALF_RANGE)
				{
					rawAccel.x = rawAccel.x - FULL_RANGE;
				}
				if (rawAccel.y >= HALF_RANGE)
				{
					rawAccel.y = rawAccel.y - FULL_RANGE;
				}
				if (rawAccel.z >= HALF_RANGE)
				{
					rawAccel.z = rawAccel.z - FULL_RANGE;
				}
				if (rawMag.x >= HALF_RANGE)
				{
					rawMag.x = rawMag.x - FULL_RANGE;
				}
				if (rawMag.y >= HALF_RANGE)
				{
					rawMag.y = rawMag.y - FULL_RANGE;
				}
				
				if (rawMag.z >= HALF_RANGE)
				{
					rawMag.z = rawMag.z - FULL_RANGE;
				}
				
				rawAccel.x = rawAccel.x * InputLPFKoeff;
				accelFiltered.x = (accelFiltered.x * (InputLPFKoeff - 1) / InputLPFKoeff + rawAccel.x * 1 / InputLPFKoeff);
				rawAccel.y = rawAccel.y * InputLPFKoeff;
				accelFiltered.y = (accelFiltered.y * (InputLPFKoeff - 1) / InputLPFKoeff + rawAccel.y * 1 / InputLPFKoeff);
				rawAccel.z = rawAccel.z * InputLPFKoeff;
				accelFiltered.z = (accelFiltered.z * (InputLPFKoeff - 1)  / InputLPFKoeff + rawAccel.z * 1 / InputLPFKoeff);
			
				rawMag.x = rawMag.x * InputLPFKoeff;
				magFiltered.x = (magFiltered.x * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.x * 1 / InputLPFKoeff);
				rawMag.y = rawMag.y * InputLPFKoeff;
				magFiltered.y = (magFiltered.y * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.y * 1 / InputLPFKoeff);
				rawMag.z = rawMag.z * InputLPFKoeff;
				magFiltered.z = (magFiltered.z * (InputLPFKoeff - 1)  / InputLPFKoeff + rawMag.z * 1 / InputLPFKoeff);
				
				
			}
			else
			{
#ifdef FXOS8700_SERIAL_DEBUG
				FXOS8700_debugMessageString("ERROR reading RAW values\r\n");				
#endif
			}
		

		}
	
	
		
		//snprintf(msg, sizeof(msg), "Read raw values successfull\r\n");	
		//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		/*
		snprintf(msg, sizeof(msg), "DATA= ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		
		snprintf(msg, sizeof(msg), "%+04i", accelFiltered.x);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	
		snprintf(msg, sizeof(msg), "%+04i", accelFiltered.y);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	
		snprintf(msg, sizeof(msg), "%+04i", accelFiltered.z);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	
		snprintf(msg, sizeof(msg), "%+04i", magFiltered.x);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	
		snprintf(msg, sizeof(msg), "%+04i", magFiltered.y);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	
		snprintf(msg, sizeof(msg), "%+04i", magFiltered.z);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		snprintf(msg, sizeof(msg), ", ");	
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		*/
		if(((accelFiltered.x - rawAccelPrevious.x) > AccCalThreshold*InputLPFKoeff) || ((rawAccelPrevious.x - accelFiltered.x) > AccCalThreshold*InputLPFKoeff) || ((accelFiltered.y - rawAccelPrevious.y) > AccCalThreshold*InputLPFKoeff) || ((rawAccelPrevious.y - accelFiltered.y) > AccCalThreshold*InputLPFKoeff) || ((accelFiltered.z - rawAccelPrevious.z) > AccCalThreshold*InputLPFKoeff) || ((rawAccelPrevious.z - accelFiltered.z) > AccCalThreshold*InputLPFKoeff))
		{
			//snprintf(msg, sizeof(msg), "not steady");	
			//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			stableDataCounter = 0;
		}
		else
		{
			stableDataCounter++;
			if (stableDataCounter == 255)
			{
				stableDataCounter--;
			}
			//snprintf(msg, sizeof(msg), "steady");	
			//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			if(stableDataCounter >= DataStableCyclesThreshold)
				
			{
			
				if (calPointsCounter == 0)
				{
					rawAccelCalibrationPoints[calPointsCounter].x = accelFiltered.x;
					rawAccelCalibrationPoints[calPointsCounter].y = accelFiltered.y;
					rawAccelCalibrationPoints[calPointsCounter].z = accelFiltered.z;
				
					rawMagCalibrationPoints[calPointsCounter].x = magFiltered.x;
					rawMagCalibrationPoints[calPointsCounter].y = magFiltered.y;
					rawMagCalibrationPoints[calPointsCounter].z = magFiltered.z;
				
					calPointsCounter++;
				
				}
				else
				{
				

				
					//_Buffer = sqrt((accelFiltered.x>>2 - rawAccelCalibrationPoints[calPointsCounter - 1].x>>2) ^ 2 + (accelFiltered.y>>2 - rawAccelCalibrationPoints[calPointsCounter - 1].y>>2) ^ 2 + (accelFiltered.z>>2 - rawAccelCalibrationPoints[calPointsCounter - 1].z>>2) ^ 2);
					//_Buffer = 12;
					_Buffer = (accelFiltered.x / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].x / InputLPFKoeff)*(accelFiltered.x / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].x / InputLPFKoeff);
					_Buffer = _Buffer + (accelFiltered.y / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].y / InputLPFKoeff)*(accelFiltered.y / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].y / InputLPFKoeff);
					_Buffer = _Buffer + (accelFiltered.z / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].z / InputLPFKoeff)*(accelFiltered.z / InputLPFKoeff  - rawAccelCalibrationPoints[calPointsCounter - 1].z / InputLPFKoeff);
					_Buffer = sqrt(_Buffer);
					//snprintf(msg, sizeof(msg), "%+04i", _Buffer);
					//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				
					if((_Buffer) >= CalibrationDistanceThreshold)
					{
						rawAccelCalibrationPoints[calPointsCounter].x = accelFiltered.x;
						rawAccelCalibrationPoints[calPointsCounter].y = accelFiltered.y;
						rawAccelCalibrationPoints[calPointsCounter].z = accelFiltered.z;
				
						rawMagCalibrationPoints[calPointsCounter].x = magFiltered.x;
						rawMagCalibrationPoints[calPointsCounter].y = magFiltered.y;
						rawMagCalibrationPoints[calPointsCounter].z = magFiltered.z;
					
#ifdef FXOS8700_SERIAL_DEBUG
						FXOS8700_debugMessageCalPointNmb(calPointsCounter);
						
#endif //FXOS8700_SERIAL_DEBUG
						calPointsCounter++;
					
					}
					if (calPointsCounter == NumberOfCalibrationPoints)
					{
#ifdef FXOS8700_SERIAL_DEBUG
						FXOS8700_debugMessageString("NumberOfCalibrationPoints reached");
						FXOS8700_debugMessageString("\r\n");
						FXOS8700_debugMessageString("Calculating approximation ellipsoid....");
						FXOS8700_debugMessageString("\r\n");
#endif //FXOS8700_SERIAL_DEBUG
					
						calibrationError = 0xFFFFFFFF;
					
					
						accelOffset.x = 0;
						accelOffset.y = 0;
						accelOffset.z = 0;
						accelScale.x = 1000;
						accelScale.y = 1000;
						accelScale.z = 1000;
					
						magOffset.x = 0;
						magOffset.y = 0;
						magOffset.z = 0;
						magScale.x = 1000;
						magScale.y = 1000;
						magScale.z = 1000;
					
					
						radius = InputLPFKoeff * 7800;  //9000
						radiusMag = InputLPFKoeff * 20;   //250
						multiplier = 2;
					
						accelOffsetResult = accelOffset;
						accelScaleResult = accelScale;
						radiusResult = radius;

						magOffsetResult = magOffset;
						magScaleResult = magScale;
						radiusResultMag = radiusMag;
						while (multiplier >= 1)
						{
							accelOffsetCurrent =  accelOffsetResult;
							accelScaleCurrent = accelScaleResult;
							radiusCurrent = radiusResult;
						
							magOffsetCurrent =  magOffsetResult;
							magScaleCurrent = magScaleResult;
							radiusCurrentMag = radiusResultMag;
						
						
						
							floatCenterCalErrorPrevious = 0;
							while (!(floatCenterCalErrorPrevious == floatCenterCalError))
							{
								floatCenterCalErrorPrevious = floatCenterCalError;
								FXOS8700_gradientCenterSearch(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 8);
							}
							floatCenterCalErrorPrevious = 0;
							while (!(floatCenterCalErrorPrevious == floatCenterCalError))
							{
								floatCenterCalErrorPrevious = floatCenterCalError;
								FXOS8700_gradientCenterSearch(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 4);
							}
							floatCenterCalErrorPrevious = 0;
							while (!(floatCenterCalErrorPrevious == floatCenterCalError))
							{
								floatCenterCalErrorPrevious = floatCenterCalError;
								FXOS8700_gradientCenterSearch(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 2);
							}
							floatCenterCalErrorPrevious = 0;
							while (!(floatCenterCalErrorPrevious == floatCenterCalError))
							{
								floatCenterCalErrorPrevious = floatCenterCalError;
								FXOS8700_gradientCenterSearch(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 1);
							}
						
							floatCalErrorPrevious = 0;
							while (!(floatCalErrorPrevious == floatCalError))
							{
								floatCalErrorPrevious = floatCalError;
								FXOS8700_gradientSearch(1, 1, 1, 1, 1, 1, VariationDepth, 80);
							}
			
							floatCalErrorPrevious = 0;
							while (!(floatCalErrorPrevious == floatCalError))
							{
								floatCalErrorPrevious = floatCalError;
								FXOS8700_gradientSearch(1, 1, 1, 1, 1, 1, VariationDepth, 10);
							}
						
							floatCalErrorPrevious = 0;
							while (!(floatCalErrorPrevious == floatCalError))
							{
								floatCalErrorPrevious = floatCalError;
								FXOS8700_gradientSearch(1, 1, 1, 1, 1, 1, VariationDepth, 1);
							}
						
							for (int8_t n = 0; n < 4; n++)
							{
								floatCalErrorPrevious = 0;
								while (!(floatCalErrorPrevious == floatCalError))
								{
									floatCalErrorPrevious = floatCalError;
									FXOS8700_gradientSearch(1, 1, 1, VariationDepth, VariationDepth, 1, VariationDepth, 1);
								}
								floatCenterCalErrorPrevious = 0;
								//floatCenterCalError = 0xffffffffffff;
								while (!(floatCenterCalErrorPrevious == floatCenterCalError))
								{
									floatCenterCalErrorPrevious = floatCenterCalError;
									FXOS8700_gradientCenterSearch(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 1);
								}
							}
						
							if (accurateMode == 1)
							{
								floatCalErrorPrevious = 0;
								while (!(floatCalErrorPrevious == floatCalError))
								{
									floatCalErrorPrevious = floatCalError;
									FXOS8700_gradientSearch(VariationDepth, VariationDepth, VariationDepth, VariationDepth, VariationDepth, 1, VariationDepth, 1);
								}	
							}
							
						
						
						
						
							///////////////////////////////////////////////////
						
						
							floatMagCenterCalErrorPrevious = 0;
							//floatMagCenterCalError = 0xffffffffffff;
							while(!(floatMagCenterCalErrorPrevious == floatMagCenterCalError))
							{
								floatMagCenterCalErrorPrevious = floatMagCenterCalError;
								FXOS8700_gradientCenterSearchMag(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 8);
							}
							floatMagCenterCalErrorPrevious = 0;
							//floatMagCenterCalError = 0xffffffffffff;
							while(!(floatMagCenterCalErrorPrevious == floatMagCenterCalError))
							{
								floatMagCenterCalErrorPrevious = floatMagCenterCalError;
								FXOS8700_gradientCenterSearchMag(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 4);
							}
							floatMagCenterCalErrorPrevious = 0;
							//floatMagCenterCalError = 0xffffffffffff;
							while(!(floatMagCenterCalErrorPrevious == floatMagCenterCalError))
							{
								floatMagCenterCalErrorPrevious = floatMagCenterCalError;
								FXOS8700_gradientCenterSearchMag(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 2);
							}
						
							floatMagCenterCalErrorPrevious = 0;
							//floatMagCenterCalError = 0xffffffffffff;
							while(!(floatMagCenterCalErrorPrevious == floatMagCenterCalError))
							{
								floatMagCenterCalErrorPrevious = floatMagCenterCalError;
								FXOS8700_gradientCenterSearchMag(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 1);
							}
						
							floatMagCalErrorPrevious = 0;
							//floatMagCalError = 0xffffffffffff;
							while(!(floatMagCalErrorPrevious == floatMagCalError))
							{
								floatMagCalErrorPrevious = floatMagCalError;
								FXOS8700_gradientSearchMag(1, 1, 1, 1, 1, 1, VariationDepth, 80);
							}
						
							floatMagCalErrorPrevious = 0;
							//floatMagCalError = 0xffffffffffff;
							while(!(floatMagCalErrorPrevious == floatMagCalError))
							{
								floatMagCalErrorPrevious = floatMagCalError;
								FXOS8700_gradientSearchMag(1, 1, 1, 1, 1, 1, VariationDepth, 10);
							}
						
							floatMagCalErrorPrevious = 0;
							//floatMagCalError = 0xffffffffffff;
							while(!(floatMagCalErrorPrevious == floatMagCalError))
							{
								floatMagCalErrorPrevious = floatMagCalError;
								FXOS8700_gradientSearchMag(1, 1, 1, 1, 1, 1, VariationDepth, 1);
							}
						
							for (int8_t m = 0; m < 4; m++)
							{
								floatMagCalErrorPrevious = 0;
								//floatMagCalError = 0xffffffffffff;
								while(!(floatMagCalErrorPrevious == floatMagCalError))
								{
									floatMagCalErrorPrevious = floatMagCalError;
									FXOS8700_gradientSearchMag(1, 1, 1, VariationDepth, VariationDepth, 1, VariationDepth, 1);
								}
						
								floatMagCenterCalErrorPrevious = 0;
								//floatMagCenterCalError = 0xffffffffffff;
								while(!(floatMagCenterCalErrorPrevious == floatMagCenterCalError))
								{
									floatMagCenterCalErrorPrevious = floatMagCenterCalError;
									FXOS8700_gradientCenterSearchMag(VariationDepth, VariationDepth, VariationDepth, 1, 1, 1, 1, 1);
								}
							}
						
							if (accurateMode == 1)
							{
								floatMagCalErrorPrevious = 0;
								//floatMagCalError = 0xffffffffffff;
								while(!(floatMagCalErrorPrevious == floatMagCalError))
								{
									floatMagCalErrorPrevious = floatMagCalError;
									FXOS8700_gradientSearchMag(VariationDepth, VariationDepth, VariationDepth, VariationDepth, VariationDepth, 1, VariationDepth, 1);
								}	
							
							}
						
						
						
						
						
							
						
				
			
							/**
							snprintf(msg, sizeof(msg), "current accel mismatch is: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatCalError, (uint16_t)((floatCalError - (uint32_t)floatCalError) * 1000.));
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "offset: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetCurrent.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetCurrent.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetCurrent.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "scale: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelScaleCurrent.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelScaleCurrent.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelScaleCurrent.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "; ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

							snprintf(msg, sizeof(msg), "radius: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", radiusCurrent);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "   multiplier: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "\r\n");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							HAL_Delay(1000);
						
							snprintf(msg, sizeof(msg), "current mag mismatch is: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatMagCalError, (uint16_t)((floatMagCalError - (uint32_t)floatMagCalError) * 1000.));
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "offset: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magOffsetCurrent.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magOffsetCurrent.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magOffsetCurrent.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "scale: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magScaleCurrent.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magScaleCurrent.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", magScaleCurrent.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "; ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

							snprintf(msg, sizeof(msg), "radius: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", radiusCurrentMag);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "   multiplier: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", multiplier);
			
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "\r\n");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							*/
							HAL_Delay(1000);
						
							//accelOffsetCalc->x = accelOffsetCurrent.x;
							//accelOffsetCalc->y = accelOffsetCurrent.y;
							//accelOffsetCalc->z = accelOffsetCurrent.z;
						
							//accelScaleCalc->x =  accelScaleCurrent.x;
							//accelScaleCalc->y =  accelScaleCurrent.y;
							//accelScaleCalc->z =  accelScaleCurrent.z;
							//accelRadiusCalc = radiusCurrent;
						
							//magOffsetCalc->x = magOffsetCurrent.x;
							//magOffsetCalc->y = magOffsetCurrent.y;
							//magOffsetCalc->z = magOffsetCurrent.z;
						
							//magScaleCalc->x = magScaleCurrent.x;
							//magScaleCalc->y = magScaleCurrent.y;
							//magScaleCalc->z = magScaleCurrent.z;
						
							//magRadiusCalc = radiusCurrentMag;
						
							output->aox = accelOffsetCurrent.x;
							output->aoy = accelOffsetCurrent.y;
							output->aoz = accelOffsetCurrent.z;
						
							output->asx =  accelScaleCurrent.x;
							output->asy =  accelScaleCurrent.y;
							output->asz =  accelScaleCurrent.z;
							output->ar = radiusCurrent;
						
							output->mox = magOffsetCurrent.x;
							output->moy = magOffsetCurrent.y;
							output->moz = magOffsetCurrent.z;
						
							output->msx = magScaleCurrent.x;
							output->msy = magScaleCurrent.y;
							output->msz = magScaleCurrent.z;
						
							output->mr = radiusCurrentMag;
						
						
							if (multiplier == 1)
							{
								multiplier = 0;
							}
							/*
							sprintf(msg, "mismatch= %d.%03d", (uint32_t)floatBuffer, (uint16_t)((floatBuffer - (uint32_t)floatBuffer) * 1000.));
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "offset: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelOffsetResult.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "scale: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						
							snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.x);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.y);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), ", ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", accelScaleResult.z);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "; ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						

							snprintf(msg, sizeof(msg), "radius: ");	
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							snprintf(msg, sizeof(msg), "%+04i", radiusResult);
							HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
							*/
						
						
							multiplier = multiplier / 2;
						}
					
					
					
						break;
					}
				
				}
				
			}
		}
	
		rawAccelPrevious.x = accelFiltered.x;
		rawAccelPrevious.y = accelFiltered.y;
		rawAccelPrevious.z = accelFiltered.z;
	
		//snprintf(msg, sizeof(msg), "\r\n");	
		//HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		//HAL_Delay(200);
		//NumberOfCalibrationPoints++;
	}
	HAL_Delay(5000);	
	
	
}

