#ifndef BMI270_H__
#define BMI270_H__

//MPU6050 Key values
#define BMI270_ADDRESS_LEN  1         //MPU6050
#define BMI270_ADDRESS     (0x68)  //MPU6050 Device Address
#define BMI270_CHIP_ID            0x24

#define BMI270_CHIP_ID_REG          0x00
#define BMI270_ERR_REG_REG          0x02
#define BMI270_STATUS_REG           0x03
#define BMI270_DATA_0_REG           0x04
#define BMI270_DATA_1_REG           0x05
#define BMI270_DATA_2_REG          0x06
#define BMI270_DATA_3_REG           0x07
#define BMI270_DATA_4_REG           0x08
#define BMI270_DATA_5_REG           0x09
#define BMI270_DATA_6_REG           0x0A
#define BMI270_DATA_7_REG           0x0B
#define BMI270_DATA_8_REG           0x0C
#define BMI270_DATA_9_REG           0x0D
#define BMI270_DATA_10_REG          0x0E
#define BMI270_DATA_11_REG          0x0F
#define BMI270_DATA_12_REG          0x10
#define BMI270_DATA_13_REG          0x11
#define BMI270_DATA_14_REG          0x12
#define BMI270_DATA_15_REG          0x13
#define BMI270_DATA_16_REG          0x14
#define BMI270_DATA_17_REG          0x15
#define BMI270_DATA_18_REG          0x16
#define BMI270_DATA_19_REG          0x17
#define BMI270_SENSORTIME_0_REG     0x18
#define BMI270_SENSORTIME_1_REG     0x19
#define BMI270_SENSORTIME_2_REG     0x1A
#define BMI270_EVENT_REG            0x1B
#define BMI270_INT_STATUS_0_REG     0x1C
#define BMI270_INT_STATUS_1_REG     0x1D
#define BMI270_SC_OUT_0_REG         0x1E
#define BMI270_SC_OUT_1_REG         0x1F
#define BMI270_WR_GEST_ACT_REG      0x20
#define BMI270_INTERNAL_STATUS  0x21
#define BMI270_TEMPERATURE_0_REG    0x22
#define BMI270_TEMPERATURE_1_REG    0x23
#define BMI270_FIFO_LENGTH_0_REG    0x24
#define BMI270_FIFO_LENGTH_1_REG    0x25
#define BMI270_FIFO_DATA_REG        0x26
#define BMI270_FEAT_PAGE_REG        0x2F
#define BMI270_FEATURES_REG         0x30
#define BMI270_ACC_CONF_REG         0x40
#define BMI270_ACC_RANGE_REG           0x41
#define BMI270_GYR_CONF_REG            0x42
#define BMI270_GYR_RANGE_REG           0x43
#define BMI270_AUX_CONF_REG            0x44
#define BMI270_FIFO_DOWNS_REG          0x45
#define BMI270_FIFO_WTM_0_REG          0x46
#define BMI270_FIFO_WTM_1_REG          0x47
#define BMI270_FIFO_CONFIG_0_REG       0x48
#define BMI270_FIFO_CONFIG_1_REG       0x49
#define BMI270_SATURATION_REG          0x4A
#define BMI270_AUX_DEV_ID_REG          0x4B
#define BMI270_AUX_IF_CONF_REG         0x4C
#define BMI270_AUX_RD_ADDR_REG         0x4D
#define BMI270_AUX_WR_ADDR_REG         0x4E
#define BMI270_AUX_WR_DATA_REG         0x4F
#define BMI270_ERR_REG_MSK_REG         0x52
#define BMI270_INT1_IO_CTRL_REG        0x53
#define BMI270_INT2_IO_CTRL_REG        0x54
#define BMI270_INT_LATCH_REG           0x55
#define BMI270_INT1_MAP_FEAT_REG       0x56
#define BMI270_INT2_MAP_FEAT_REG       0x57
#define BMI270_INT_MAP_DATA_REG        0x58
#define BMI270_INIT_CTRL_REG           0x59
#define BMI270_INIT_ADDR_0_REG         0x5B
#define BMI270_INIT_ADDR_1_REG         0x5C
#define BMI270_INIT_DATA_REG           0x5E
#define BMI270_INTERNAL_ERROR_REG      0x5F
#define BMI270_AUX_IF_TRIM_REG         0x68
#define BMI270_GYR_CRT_CONF_REG        0x69
#define BMI270_NVM_CONF_REG            0x6A
#define BMI270_IF_CONF_REG             0x6B
#define BMI270_DRV_REG                 0x6C
#define BMI270_ACC_SELF_TEST_REG       0x6D
#define BMI270_GYR_SELF_TEST_AXES_REG  0x6E
#define BMI270_NV_CONF_REG             0x70
#define BMI270_OFFSET_0_REG            0x71
#define BMI270_OFFSET_1_REG            0x72
#define BMI270_OFFSET_2_REG            0x73
#define BMI270_OFFSET_3_REG            0x74
#define BMI270_OFFSET_4_REG            0x75
#define BMI270_OFFSET_5_REG            0x76
#define BMI270_OFFSET_6_REG            0x77
#define BMI270_PWR_CONF_REG            0x7C
#define BMI270_PWR_CTRL_REG            0x7D
#define BMI270_CMD_REG                 0x7E



typedef struct BMI270 
{
  const nrfx_twi_t *mHandle;

  bool mTransferDone;
  bool initialised;
} BMI270;



bool bmi270_init(BMI270 *sensor, const nrfx_twi_t *m_twi);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool bmi270_register_write(BMI270 *sensor, uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool bmi270_register_read(BMI270 *sensor, uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

bool bmi270_write_bit(BMI270 *sensor, uint8_t register_address, uint8_t bitNumber, uint8_t data);

bool bmi270_read_bit(BMI270 *sensor, uint8_t register_address, uint8_t bitNumber, uint8_t *data);


bool bmi270_ReadAcc(BMI270 *sensor, int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );
bool bmi270_ReadTemp(BMI270 *sensor, float *pTemp );

bool bmi270_VerifyChipId(BMI270 *sensor);

bool bmi270_EnableAccelerometer(BMI270 *sensor, bool enable);

bool bmi270_EnableAdvancedPowerSave(BMI270 *sensor, bool enable);

bool bmi270_EnableFastPowerUp(BMI270 *sensor, bool enable);

bool bmi270_GetInternalStatus(BMI270 *sensor, uint8_t *statusMessage);

bool bmi270_WriteConfig(BMI270 *sensor);



#endif


