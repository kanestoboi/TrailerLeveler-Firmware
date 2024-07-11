#ifndef MPU6050_H__
#define MPU6050_H__

//MPU6050 Key values
#define MPU6050_ADDRESS_LEN  1         //MPU6050
#define MPU6050_ADDRESS     (0xD0>>1)  //MPU6050 Device Address
#define MPU6050_WHO_AM_I     0x68U     //MPU6050 ID

//MPU6050 Registers addresses, see datasheet for more info and each register's function
#define MPU6050_AUX_VDDIO_REG			0x00
#define MPU6050_YG_OFFS_TC_REG			0x01
#define MPU6050_ZG_OFFS_TC_REG			0x02
#define MPU6050_X_FINE_GAIN_REG			0x03
#define MPU6050_Y_FINE_GAIN_REG			0x04
#define MPU6050_Z_FINE_GAIN_REG			0x05
#define MPU6050_XA_OFFS_H_REG			0x06
#define MPU6050_XA_OFFS_L_TC_REG		0x07
#define MPU6050_YA_OFFS_H_REG			0x08
#define MPU6050_YA_OFFS_L_TC_REG		0x09
#define MPU6050_ZA_OFFS_H_REG			0x0A
#define MPU6050_ZA_OFFS_L_TC_REG		0x0B
#define MPU6050_SELF_TEST_X_REG			0x0D
#define MPU6050_SELF_TEST_Y_REG			0x0E
#define MPU6050_SELF_TEST_Z_REG			0x0F
#define MPU6050_SELF_TEST_A_REG			0x10
#define MPU6050_XG_OFFS_USRH_REG		0x13
#define MPU6050_XG_OFFS_USRL_REG		0x14
#define MPU6050_YG_OFFS_USRH_REG		0x15
#define MPU6050_YG_OFFS_USRL_REG		0x16
#define MPU6050_ZG_OFFS_USRH_REG		0x17
#define MPU6050_ZG_OFFS_USRL_REG		0x18
#define MPU6050_SAMPLE_RATE_REG			0x19
#define MPU6050_CONFIG_REG          	0x1A
#define MPU6050_GYRO_CONFIG_REG			0x1B
#define MPU6050_ACCEL_CONFIG_REG    	0x1C
#define MPU6050_FF_THR_REG    			0x1D
#define MPU6050_FF_DUR_REG    			0x1E
#define MPU6050_MOT_THR_REG				0x1F
#define MPU6050_MOT_DUR_REG         	0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU6050_ZMOT_THR_REG        	0x21  // Zero-motion detection threshold bits [7:0]
#define MPU6050_ZRMOT_DUR_REG       	0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define MPU6050_FIFO_EN_REG         	0x23
#define MPU6050_I2C_MST_CTRL_REG		0x24
#define MPU6050_I2C_SLV0_ADDR_REG   	0x25
#define MPU6050_I2C_SLV0_REG			0x26
#define MPU6050_I2C_SLV0_CTRL_REG   	0x27
#define MPU6050_I2C_SLV1_ADDR_REG   	0x28
#define MPU6050_I2C_SLV1_REG			0x29
#define MPU6050_I2C_SLV1_CTRL_REG   	0x2A
#define MPU6050_I2C_SLV2_ADDR_REG   	0x2B
#define MPU6050_I2C_SLV2_REG			0x2C
#define MPU6050_I2C_SLV2_CTRL_REG   	0x2D
#define MPU6050_I2C_SLV3_ADDR_REG   	0x2E
#define MPU6050_I2C_SLV3_REG			0x2F
#define MPU6050_I2C_SLV3_CTRL_REG   	0x30
#define MPU6050_I2C_SLV4_ADDR_REG   	0x31
#define MPU6050_I2C_SLV4_REG			0x32
#define MPU6050_I2C_SLV4_DO_REG			0x33
#define MPU6050_I2C_SLV4_CTRL_REG   	0x34
#define MPU6050_I2C_SLV4_DI_REG			0x35
#define MPU6050_I2C_MST_STA_REG			0x36
#define MPU6050_INT_BP_CFG_REG			0x37
#define MPU6050_INT_EN_REG				0x38
#define MPU6050_DMP_INT_STATUS_REG		0x39
#define MPU6050_INT_STA_REG				0x3A
#define MPU6050_ACC_OUT_REG         	0x3B
#define MPU6050_ACCEL_XOUT_H_REG		0x3B
#define MPU6050_ACCEL_XOUT_L_REG		0x3C
#define MPU6050_ACCEL_YOUT_H_REG		0x3D
#define MPU6050_ACCEL_YOUT_L_REG		0x3E
#define MPU6050_ACCEL_ZOUT_H_REG		0x3F
#define MPU6050_ACCEL_ZOUT_L_REG		0x40
#define MPU6050_TEMP_OUT_H_REG			0x41
#define MPU6050_TEMP_OUT_L_REG			0x42
#define MPU6050_GYRO_OUT_REG        	0x43
#define MPU6050_GYRO_XOUT_H_REG			0x43
#define MPU6050_GYRO_XOUT_L_REG			0x44
#define MPU6050_GYRO_YOUT_H_REG			0x45
#define MPU6050_GYRO_YOUT_L_REG			0x46
#define MPU6050_GYRO_ZOUT_H_REG			0x47
#define MPU6050_GYRO_ZOUT_L_REG			0x48
#define MPU6050_EXT_SENS_DATA_00_REG	0x49
#define MPU6050_EXT_SENS_DATA_01_REG	0x4A
#define MPU6050_EXT_SENS_DATA_02_REG	0x4B
#define MPU6050_EXT_SENS_DATA_03_REG	0x4C
#define MPU6050_EXT_SENS_DATA_04_REG	0x4D
#define MPU6050_EXT_SENS_DATA_05_REG	0x4E
#define MPU6050_EXT_SENS_DATA_06_REG	0x4F
#define MPU6050_EXT_SENS_DATA_07_REG	0x50
#define MPU6050_EXT_SENS_DATA_08_REG	0x51
#define MPU6050_EXT_SENS_DATA_09_REG	0x52
#define MPU6050_EXT_SENS_DATA_10_REG	0x53
#define MPU6050_EXT_SENS_DATA_11_REG	0x54
#define MPU6050_EXT_SENS_DATA_12_REG	0x55
#define MPU6050_EXT_SENS_DATA_13_REG	0x56
#define MPU6050_EXT_SENS_DATA_14_REG	0x57
#define MPU6050_EXT_SENS_DATA_15_REG	0x58
#define MPU6050_EXT_SENS_DATA_16_REG	0x59
#define MPU6050_EXT_SENS_DATA_17_REG	0x5A
#define MPU6050_EXT_SENS_DATA_18_REG	0x5B
#define MPU6050_EXT_SENS_DATA_19_REG	0x5C
#define MPU6050_EXT_SENS_DATA_20_REG	0x5D
#define MPU6050_EXT_SENS_DATA_21_REG	0x5E
#define MPU6050_EXT_SENS_DATA_22_REG	0x5F
#define MPU6050_EXT_SENS_DATA_23_REG	0x60
#define MPU6050_MOT_DETECT_STATUS_REG	0x61
#define MPU6050_I2C_SLV0_DO_REG			0x63
#define MPU6050_I2C_SLV1_DO_REG			0x64
#define MPU6050_I2C_SLV2_DO_REG			0x65
#define MPU6050_I2C_SLV3_DO_REG			0x66
#define MPU6050_I2C_MST_DELAY_REG   	0x67
#define MPU6050_SIG_PATH_RST_REG		0x68
#define MPU6050_MOT_DETECT_CTRL			0x69
#define MPU6050_USER_CTRL_REG			0x6A
#define MPU6050_PWR_MGMT1_REG			0x6B
#define MPU6050_PWR_MGMT2_REG			0x6C
#define MPU6050_BANK_SEL_REG			0x6D
#define MPU6050_MEM_START_ADDR_REG		0x6E
#define MPU6050_MEM_R_W_REG				0x6F
#define MPU6050_DMP_CFG_1_REG			0x71
#define MPU6050_FIFO_COUNTH_REG			0x72
#define MPU6050_FIFO_COUNTL_REG			0x73
#define MPU6050_FIFO_RW_REG				0x74
#define MPU6050_WHO_AM_I_REG			0x75

typedef struct MPU6050 
{
  const nrfx_twi_t *mHandle;
  bool initialised;
} MPU6050;

enum MPU6050_INTERRUPT_SOURCE 
{
        DISABLE_ALL_INTERRUPTS = 0,
        FF_EN = 0x80,
	MOT_EN = 0x40U,
        ZMOT_EN = 0x20,
	FIFO_OFLOW_EN = 0x10,
	I2C_MST_INT_EN = 0x08,
        PLL_RDY_INT = 0x04,
        DMP_INT = 0x02,
	DATA_RDY_EN = 0x01
};

typedef enum MPU6050_WAKE_UP_FREQUENCY 
{
  WAKE_UP_1_25_HZ = 0,
  WAKE_UP_2_5_HZ = 1,
  WAKE_UP_5_HZ = 2,
  WAKE_UP_10_HZ = 3,
} MPU6050_WAKE_UP_FREQUENCY;

bool mpu6050_init(MPU6050 *sensor, const nrfx_twi_t *m_twi);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mpu6050_register_write(MPU6050 *sensor, uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mpu6050_register_read(MPU6050 *sensor, uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

bool mpu6050_write_bit(MPU6050 *sensor, uint8_t register_address, uint8_t bitNumber, uint8_t data);

/**
  @brief Function for reading and verifying Analog Devices ID.
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool mpu6050_verify_product_id(MPU6050 *sensor);

/**
  @brief Function for reading and verifying the MPU6050 MEMS ID.
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool mpu6050_verify_mems_id(MPU6050 *sensor);

/**
  @brief Function for reading and verifying the MPU6050 Part ID.
  @retval true ID is what was expected
  @retval false ID was not what was expected
*/  
bool mpu6050_verify_part_id(MPU6050 *sensor);

bool mpu6050_ReadAcc(MPU6050 *sensor, int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );
bool mpu6050_ReadTemp(MPU6050 *sensor, int16_t *pTemp );
bool mpu6050_DataReady(MPU6050 *sensor);

bool mpu6050_SetMotionDetectionThreshold(MPU6050 *sensor, uint8_t threshold);

bool mpu6050_EnableInterrupt(MPU6050 *sensor, uint8_t source);

bool mpu6050_ConfigureInterruptPin(MPU6050 *sensor, uint8_t configuration);

bool mpu6050_SetMotionDetectionDuration(MPU6050 *sensor, uint8_t duration);

bool mpu6050_SetAccelerometerPowerOnDelay(MPU6050 *sensor, uint8_t delay);

bool mpu6050_SetFreefallDetectionCounterDecrement(MPU6050 *sensor, uint8_t decrement);

bool mpu6050_SetMotionDetectionCounterDecrement(MPU6050 *sensor, uint8_t decrement);

bool mpu6050_SetTemperatureFIFOEnabled(MPU6050 *sensor, bool enabled);

bool mpu6050_SetSleepDisabled(MPU6050 *sensor, bool disabled);

bool mpu6050_SetTemperatureDisabled(MPU6050 *sensor, bool disabled);

bool mpu6050_SetCycleEnabled(MPU6050 *sensor, bool enabled);

bool mpu6050_SetCycleFrequency(MPU6050 *sensor, MPU6050_WAKE_UP_FREQUENCY frequency);

bool mpu6050_Reset(MPU6050 *sensor);


#endif


