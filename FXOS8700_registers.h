// FXOS8700 I2C address
#define FXOS8700CQ_SLAVE_ADDR	0x1E // with pins SA0=0, SA1=0


// FXOS8700 internal register addresses
#define FXOS8700CQ_STATUS		0x00
#define FXOS8700CQ_WHOAMI		0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1	0x2A
#define FXOS8700CQ_M_CTRL_REG1	0x5B
#define FXOS8700CQ_M_CTRL_REG2	0x5C
#define FXOS8700CQ_WHOAMI_VAL	0xC7
// number of bytes to be read from the FXOS8700
#define FXOS8700CQ_READ_LEN		13 // status plus 6 channels = 13 bytes

/////////////////////////////////////////////////////////////////
#define FXOS8700CQ_CTRL_REG1_ACTIVE						(1 << 0)
#define FXOS8700CQ_CTRL_REG1_INACTIVE					(0 << 0)

#define FXOS8700CQ_CTRL_REG1_NORMAL_READ_MODE			(0 << 1)
#define FXOS8700CQ_CTRL_REG1_FAST_READ_MODE				(1 << 1)

#define FXOS8700CQ_CTRL_REG1_NORMAL_NOISE_MODE			(0 << 2)
#define FXOS8700CQ_CTRL_REG1_REDUCED_NOISE_MODE			(1 << 2)

#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_0			(0 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_1			(1 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_2			(2 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_3			(3 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_4			(4 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_5			(5 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_6			(6 << 3)
#define FXOS8700CQ_CTRL_REG1_OUTPUT_DATA_RATE_7			(7 << 3)


#define FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_50Hz			(0 << 6)
#define FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_12_5Hz			(1 << 6)
#define FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_6_25Hz			(2 << 6)
#define FXOS8700CQ_CTRL_REG1_AUTO_WKP_F_1_56Hz			(3 << 6)
/////////////////////////////////////////////////////////////////

#define FXOS8700CQ_M_CTRL_REG1_ACCEL_ONLY_MODE			(0 << 0)
#define FXOS8700CQ_M_CTRL_REG1_MAG_ONLY_MODE			(1 << 0)
#define FXOS8700CQ_M_CTRL_REG1_HYBRID_MODE				(3 << 0)

#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_0		(0 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_1		(1 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_2		(2 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_3		(3 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_4		(4 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_5		(5 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_6		(6 << 2)
#define FXOS8700CQ_M_CTRL_REG1_OXERSAMPLE_RATIO_7		(7 << 2)

#define FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_MEASURE_OFF	(0 << 5)
#define FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_MEASURE_ON	(1 << 5)

#define FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_RST_OFF		(0 << 6)
#define FXOS8700CQ_M_CTRL_REG1_ONE_SHOT_M_RST_ON		(1 << 6)

#define FXOS8700CQ_M_CTRL_REG1_AUTO_CALIBRATION_OFF		(0 << 7)
#define FXOS8700CQ_M_CTRL_REG1_AUTO_CALIBRATION_ON		(1 << 7)

///////////////////////////////////////////////////////////////////

#define FXOS8700CQ_M_CTRL_REG2_MAG_RST_EVERY_1_CYCLE	(0 << 0)
#define FXOS8700CQ_M_CTRL_REG2_MAG_RST_EVERY_16_CYCLE	(1 << 0)
#define FXOS8700CQ_M_CTRL_REG2_MAG_RST_EVERY_512_CYCLES	(2 << 0)
#define FXOS8700CQ_M_CTRL_REG2_MAG_RST_DISABLED			(3 << 0)

#define FXOS8700CQ_M_CTRL_REG2_MAG_MAXIM_RST_OFF		(0 << 2)
#define FXOS8700CQ_M_CTRL_REG2_MAG_MAXIM_RST_ON			(1 << 2)

#define FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_THD_OFF		(0 << 3)
#define FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_THD_ON		(1 << 3)

#define FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_OFF			(0 << 4)
#define FXOS8700CQ_M_CTRL_REG2_MAG_MIN_MAX_ON			(1 << 4)

#define FXOS8700CQ_M_CTRL_REG2_MAG_AUTOINCREMENT_OFF	(0 << 5)
#define FXOS8700CQ_M_CTRL_REG2_MAG_AUTOINCREMENT_ON		(1 << 5)
/////////////////////////////////////////////////////////////////

#define FXOS8700CQ_XYZ_DATA_CFG_RANGE_0_244mg_per_LSB	(0 << 0)
#define FXOS8700CQ_XYZ_DATA_CFG_RANGE_0_488mg_per_LSB	(1 << 0)
#define FXOS8700CQ_XYZ_DATA_CFG_RANGE_0_976mg_per_LSB	(2 << 0)

#define FXOS8700CQ_XYZ_DATA_CFG_HPF_OFF					(0 << 4)
#define FXOS8700CQ_XYZ_DATA_CFG_HPF_ON					(1 << 4)

////////////////////////////////////////////////////////////////

