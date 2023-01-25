#include "stm32l0xx_hal.h"
#include "math.h"

#define	FXOS8700_SERIAL_DEBUG
#define FXOS8700_OK						0
#define FXOS8700_COMMUNICATION_ERROR	1




typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} SRAWDATA;

typedef struct
{
	int32_t aox;
	int32_t aoy;
	int32_t aoz;
	int32_t asx;
	int32_t asy;
	int32_t asz;
	int32_t ar;
	
	int32_t mox;
	int32_t moy;
	int32_t moz;
	int32_t msx;
	int32_t msy;
	int32_t msz;
	int32_t mr;
} CALIBRATIONRESULT;



/**

* @brief Checks if FXOS8700 chip is online

* @param None

* @return returns 0 if not online; returns 1 if chip response detected

*/
uint8_t FXOS8700_checkResponse();


/**

* @ brief Reads data from FXOS8700

* @ param Pointers to data structures to write accelerometer and magnitometer raw 3D data arrays to

* @ return Stores acc and mag datasubsequently.The corresponding subsequent mode must be set first.This may be done by calling the FXOS8700_init() function

*/	
uint8_t FXOS8700_readAccelAndMag(SRAWDATA *pAccelData, SRAWDATA *pMagnData);


/**

* @ brief Reads magnetic heading from FXOS8700

* @ param As a parameter uses a structure holding previously saved calibration data

* @ return Returns magnetic heading in degrees multiplied by InputLPFKoeff. For instance, if InputLPFKoeff=100 and magnetic heading is 45 deg, the function will return 4500. This measure helps not no use floating point return value. So, the more InputLPFKoeff is the more accuracy is and the slower time response is.

*/	
int32_t FXOS8700_getMagneticHeading(CALIBRATIONRESULT calibData, uint8_t *ERROR_CODE);

/**

* @ brief Tests FXOS8700 for debug purposes

* @ param None

* @ return Returns none. Produces serial debug messages. 

*/
uint8_t FXOS8700_test();

/**

* @ brief Initializes an FXOS8700 chip. sets the following: 200-Hz hybrid mode. Both accelerometer and magnetometer data are provided at the 200 - Hz rate

* @ param None

* @ return Returns FXOS8700_OK if an FXOS8700 responds all the  commands and FXOS8700_COMMUNICATION_ERROR if does not respond at least one command

*/
uint8_t FXOS8700_init();


/**

* @ brief Calculates calibration error as a sum mismatch in approximation ellipsoid formula

* @ param point- a point 3D coordinates
* offset- an 3D-coordinates-expressed offset in ellipsoid formula
* scale- a scale coefficient in ellipsoid formula
* radius- a radius of an ellipsoid in ellipsoid formula

* @ return Returns 1 if an FXOS8700 responds commands and 0 if not

*/
float_t mismatch(SRAWDATA point, SRAWDATA offset, SRAWDATA scale, uint32_t radius);


/**

* @ brief Searches for a best fit for ellipsoid formula coefficients within given ranges for the magnetometer data

* @ param radX - a range within witch the dedicated ellipsoid formula coefficient will be varied in order to find the best solution for an ellipsoid approximation

* @ return Returns none. Stores the best found solution to the current ellipsoid coefficients variables
*/ 
void FXOS8700_gradientSearchMag(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul);


/**

* @ brief Searches for a best fit for ellipsoid formula coefficients within given ranges for the accelerometer data

* @ param radX - a range within witch the dedicated ellipsoid formula coefficient will be varied in order to find the best solution for an ellipsoid approximation

* @ return Returns none. Stores the best found solution to the current ellipsoid coefficients variables
*/ 
void FXOS8700_gradientSearch(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul);


/**

* @ brief Searches for a best fit for ellipsoid formula coefficients regarding ellipsoid center position mismatch within given ranges for the accelerometer data.
* 
*This function is designed to make the solution process more stable. It is called from time to time in order not to miss center point while searching for a best fit solution.

* @ param radX - a range within witch the dedicated ellipsoid formula coefficient will be varied in order to find the best solution for an ellipsoid approximation

* @ return Returns none. Stores the best found solution to the current ellipsoid coefficients variables
*/ 
void FXOS8700_gradientCenterSearch(uint16_t rad1, uint16_t rad2, uint16_t rad3, uint16_t rad4, uint16_t rad5, uint16_t rad6, uint16_t rad7, uint8_t mul);

/**

* @ brief Calibrates both magnetometer and accelerometer.
* A high level funcition whitch envolves other functions like FXOS8700_gradientSearch, collect data points for calibration and returns an approximation ellipsoid formula coeficcients.

* @ param accurateMode - sets calculation algogithm.	1- more accurate and time consupting algorithm
*														0- less accurate and fast algorithm
*														Set to 0 with many calibration points
*														Set to 1 with not many (like 10) calibration points.

* @ return Returns an approximation ellipsoid formula coefficients via the calibration result variable
*/ 
void FXOS8700_calibrate(CALIBRATIONRESULT *output, int8_t accurateMode);


/**

* @ Prints an integer value to the serial port in HEX format

* @ param portNumber - a serial port number to print the message to

* @ return None
*/

void FXOS8700_debugMessageAsHEX(int8_t message);

/**

* @ Prints a debug report about a better mismatch found

* @ param None

* @ return None
*/

void FXOS8700_debugMessageBetterMagMismatch();

/**

* @ Prints a debug report about a better mismatch found

* @ param None

* @ return None
*/

void FXOS8700_debugMessageBetterAccelMismatch();


/**
* @ Prints a debug report about a new calibration point added

* @ param calPointNumber - calibration point added number

* @ return None
*/

void FXOS8700_debugMessageCalPointNmb(uint8_t calpointNmb);

/**
* @ Prints a string debug message

* @ param message[64] - char array to print to the serial port

* @ return None
*/

void FXOS8700_debugMessageString(char message[]);