#include <linux/types.h>
#include <time.h>

#ifndef ICM20602_H_
#define ICM20602_H_

typedef struct _IMUData {
	struct timespec ts;

	float gyro[3];
	float accl[3];
	float temp;
} IMUData_t;

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS     9.80665f

// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_ACCEL_XOUT_H			0x3B
#define MPUREG_ACCEL_XOUT_L			0x3C
#define MPUREG_ACCEL_YOUT_H			0x3D
#define MPUREG_ACCEL_YOUT_L			0x3E
#define MPUREG_ACCEL_ZOUT_H			0x3F
#define MPUREG_ACCEL_ZOUT_L			0x40
#define MPUREG_TEMP_OUT_H				0x41
#define MPUREG_TEMP_OUT_L				0x42
#define MPUREG_GYRO_XOUT_H			0x43
#define MPUREG_GYRO_XOUT_L			0x44
#define MPUREG_GYRO_YOUT_H			0x45
#define MPUREG_GYRO_YOUT_L			0x46
#define MPUREG_GYRO_ZOUT_H			0x47
#define MPUREG_GYRO_ZOUT_L			0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1       0x11
#define MPUREG_ICM_UNDOC1_VALUE 	0xc9
// REG.26: Configuration
#define MPUREG_CONFIG			0x1A
#define BITS_DLPF_CFG_250HZ 		0x00
#define BITS_DLPF_CFG_176HZ			0x01
#define BITS_DLPF_CFG_92HZ			0x02
#define BITS_DLPF_CFG_41HZ			0x03
#define BITS_DLPF_CFG_20HZ			0x04
#define BITS_DLPF_CFG_10HZ			0x05
#define BITS_DLPF_CFG_5HZ				0x06
#define BITS_DLPF_CFG_NOLPF			0x07
#define BITS_DLPF_CFG_MASK			0x07
// REG.27: Gyroscope configuration
#define MPUREG_GYRO_CONFIG 0x1B
#define BITS_FS_250DPS					0x00
#define BITS_FS_500DPS					0x08
#define BITS_FS_1000DPS					0x10
#define BITS_FS_2000DPS					0x18
#define BITS_FS_MASK						0x18
// REG.28: Accelerometer configuration
#define MPUREG_ACCEL_CONFIG 0x1C
#define BITS_AFS_02G						0x00
#define BITS_AFS_04G						0x08
#define BITS_AFS_08G						0x10
#define BITS_AFS_16G						0x18
#define BITS_AFS_MASK						0x18
// REG.29: Accelerometer configuration for low noise mode
#define MPUREG_ACCEL_CONFI2 0x1D
#define BITS_ADLPF_CFG_218HZ		0x00
#define BITS_ADLPF_CFG_99HZ			0x02
#define BITS_ADLPF_CFG_44HZ			0x03
#define BITS_ADLPF_CFG_21HZ			0x04
#define BITS_ADLPF_CFG_10HZ			0x05
#define BITS_ADLPF_CFG_5HZ			0x06
#define BITS_ADLPF_CFG_NOLPF		0x07
#define BITS_ADLPF_CFG_MASK			0x07

#define MPUREG_INT_PIN_CFG			0x37
#define BIT_INT_ANYRD_2CLEAR		0x10
#define MPUREG_INT_STATUS			0x3A
#define BIT_RDY_INT							0x01
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

// Product ID Description for ICM20602
// high 4 bits 	low 4 bits
// Product Name	Product Revision

#define ICM20602ES_REV_C4		0x14
#define ICM20602ES_REV_C5		0x15
#define ICM20602ES_REV_D6		0x16
#define ICM20602ES_REV_D7		0x17
#define ICM20602ES_REV_D8		0x18
#define ICM20602_REV_C4			0x54
#define ICM20602_REV_C5			0x55
#define ICM20602_REV_D6			0x56
#define ICM20602_REV_D7			0x57
#define ICM20602_REV_D8			0x58
#define ICM20602_REV_D9			0x59
#define ICM20602_REV_D10			0x5A

#define ICM20602_ACCEL_DEFAULT_RANGE_G				8
#define ICM20602_ACCEL_DEFAULT_RATE					1000
#define ICM20602_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ICM20602_GYRO_DEFAULT_RANGE_G				8
#define ICM20602_GYRO_DEFAULT_RATE					1000
#define ICM20602_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ICM20602_DEFAULT_ONCHIP_FILTER_FREQ			80

#ifdef __cplusplus
extern "C" {
#endif

int icm20602_open(const char* dev);
int icm20602_init(int fd);
/** @brief  Read IMU data from spi 
* @param[in] fd  File descriptor
* @param[out] report  IMU data
* @return  The IMU data is new (1) or not (0). */
int icm20602_read(int fd, IMUData_t *report);
int icm20602_close(int fd);

#ifdef __cplusplus
}
#endif

#endif /* ICM20602_H_ */
