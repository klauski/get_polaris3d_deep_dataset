
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>
#include "icm20602.h"

#define ICM20602_LOW_BUS_SPEED			10*1000*1000
#define ICM20602_HIGH_BUS_SPEED			11*1000*1000 /* will be rounded to 10.4 MHz, within margins for MPU6K */
#define DIR_READ			              0x80
#define DIR_WRITE			              0x00

#define swap_uint16_t(x)((x & 0xff) << 8) | ((x & 0xff00) >> 8)

static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = ICM20602_LOW_BUS_SPEED;
static uint16_t delay = 0;
static unsigned int sample_rate = 200;
static const float temp_zero = 25;
static const float temp_sensitivity = 1 / 326.8f; 
static float gyro_fscale;
static float accel_fscale;

static int transfer(int fd, uint8_t* tx, uint8_t* rx, uint8_t size)
{
  struct spi_ioc_transfer tr = {
    .tx_buf        = (unsigned long)tx,
    .rx_buf        = (unsigned long)rx,
    .len           = size,
    .delay_usecs   = delay,
    .speed_hz      = speed,
    .bits_per_word = bits,
  };

  return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

static int transfer2(int fd, uint8_t* tx, uint8_t txlen, uint8_t* rx, uint8_t rxlen)
{
  struct spi_ioc_transfer tr[] = {
		{
	    .tx_buf        = (uint64_t)tx,
	    .rx_buf        = 0,
	    .len           = txlen,
	    .delay_usecs   = delay,
	    .speed_hz      = speed,
	    .bits_per_word = bits,
		},
		{
	    .tx_buf        = 0,
	    .rx_buf        = (uint64_t)rx,
	    .len           = rxlen,
	    .delay_usecs   = delay,
	    .speed_hz      = speed,
	    .bits_per_word = bits,
		},
  };
//TODO: critical section??
  return ioctl(fd, SPI_IOC_MESSAGE(2), tr);
}


static uint8_t read_reg(int fd, unsigned reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	transfer(fd, cmd, cmd, sizeof(cmd));

	return cmd[1];
}

static uint16_t read_reg16(int fd, unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	transfer(fd, cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

static uint8_t read_multiple_regs(int fd, unsigned reg, uint8_t *buffer, uint8_t buffer_len)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	transfer2(fd, cmd, 1, buffer, buffer_len);

	return cmd[1];
}


static void write_reg(int fd, unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(fd, cmd, NULL, sizeof(cmd));
}

/*
 set the DLPF filter frequency. This affects both accel and gyro.
 */
static void set_dlpf_filter(int spi, uint16_t frequency_hz) {
	uint8_t filter, filter2;

	// choose next highest filter frequency available (DLPF)
	if (frequency_hz <= 5) {
		filter = BITS_DLPF_CFG_5HZ;
	} else if (frequency_hz <= 10) {
		filter = BITS_DLPF_CFG_10HZ;
	} else if (frequency_hz <= 20) {
		filter = BITS_DLPF_CFG_20HZ;
	} else if (frequency_hz <= 41) {
		filter = BITS_DLPF_CFG_41HZ;
	} else if (frequency_hz <= 92) {
		filter = BITS_DLPF_CFG_92HZ;
	} else if (frequency_hz <= 176) {
		filter = BITS_DLPF_CFG_176HZ;
	} else if (frequency_hz <= 250) {
		filter = BITS_DLPF_CFG_250HZ;
	} else {
		filter = BITS_DLPF_CFG_NOLPF;
	}
	// choose next highest filter frequency available (A-DLPF)
	if (frequency_hz <= 5) {
		filter2 = BITS_ADLPF_CFG_5HZ;
	} else if (frequency_hz <= 10) {
		filter2 = BITS_ADLPF_CFG_10HZ;
	} else if (frequency_hz <= 21) {
		filter2 = BITS_ADLPF_CFG_21HZ;
	} else if (frequency_hz <= 44) {
		filter2 = BITS_ADLPF_CFG_44HZ;
	} else if (frequency_hz <= 99) {
		filter2 = BITS_ADLPF_CFG_99HZ;
	} else if (frequency_hz <= 218) {
		filter2 = BITS_ADLPF_CFG_218HZ;
	} else {
		filter2 = BITS_ADLPF_CFG_NOLPF;
	}

	write_reg(spi, MPUREG_CONFIG, filter);
	write_reg(spi, MPUREG_ACCEL_CONFI2, filter2);
}

static void set_sample_rate(int spi, uint16_t desired_sample_rate_hz) {
	uint8_t div = 1000 / desired_sample_rate_hz;
	if (div > 200)
		div = 200;
	if (div < 1)
		div = 1;
	write_reg(spi, MPUREG_SMPLRT_DIV, div - 1);
	sample_rate = 1000 / div;
	printf("Sample Rate: %d\n", sample_rate);
}

int icm20602_open(const char* dev)
{
  uint8_t val = 0;
  int fd, ret;

  if (dev == NULL) { printf("invalid parameter\n"); return -1; }

  fd = open(dev, O_RDWR);
  if (fd < 0) { printf("can't open device %s\n", dev); return -1; }
  // spi mode
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1) { printf("can't set spi mode\n"); return -1; }
  ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
  if (ret == -1) { printf("can't get spi mode\n"); return -1; }
  // bits per word
  ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1) { printf("can't set bits per word\n"); return -1; }
  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1) { printf("can't get bits per word\n"); return -1; }
  // max speed (Hz)
  ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1) { printf("can't set max speed hz\n"); return -1; }
  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1) { printf("can't get max speed hz\n"); }
  // WHIAMI: 0x12 (default)
  val = read_reg(fd, MPUREG_WHOAMI);
  printf("WHOAMI : %02x\n", val);
  return fd;
}

int icm20602_init(int spi)
{
	const int delay_us = 10 * 1000;
	if (spi <= 0) {
		printf("Invalid Param\n");
		return -1;
	}
	//reset
	write_reg(spi, MPUREG_PWR_MGMT_1, BIT_H_RESET);
	usleep(10*delay_us); // 100ms

	//sleep mode off
	write_reg(spi, MPUREG_PWR_MGMT_1, 0x00); 
//	write_reg(spi, MPUREG_PWR_MGMT_1, 0x08); // disable temperature sensor
	usleep(10*delay_us);

	//disable I2C
	//TODO: is it necessary??
	write_reg(spi, MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
	usleep(delay_us);

	// SAMPLE RATE. Only effective when DLPF_CFG < 250 Hz
	set_sample_rate(spi, sample_rate);
	usleep(delay_us);
	// DLPF = 250 Hz (low pass filter)
	set_dlpf_filter(spi, ICM20602_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(delay_us);
	// Gyroscope full scale: 2000 dps, FCHOICE_B(00): DLPF is enabled.
	write_reg(spi, MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	gyro_fscale = (0.0174532f / 16.4f); // 16.4 LSB/(dps)
	usleep(delay_us);
	// Accelerometer full scale: 16g
	write_reg(spi, MPUREG_ACCEL_CONFIG, BITS_AFS_16G);
	accel_fscale = GRAVITY_MSS / 2048.f; // 2048 LSB/g
	usleep(delay_us);
	// Write undocumented register
//	write_reg(spi, MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE);
//	usleep(delay_us);
	// INT CFG => Interrupt on Data Ready
//	write_reg(spi, MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
//	usleep(delay_us);

//	write_reg(spi, MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);
	write_reg(spi, MPUREG_INT_PIN_CFG, 0x0); // Nothing being set.
	usleep(delay_us);

	return 0;
}

int icm20602_read(int fd, IMUData_t *x)
{
	int result;

	// the axis order is changed to that of datasheet
#pragma pack(push, 1)
	struct mpu_report {
		uint8_t		status;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} mpu_raw_report = {0};
#pragma pack(pop)

	// change this to be a dma transfer
//#define FAST_METHOD 0
#if defined(FAST_METHOD)
	read_multiple_regs(fd, MPUREG_INT_STATUS, ((uint8_t *)&mpu_raw_report), sizeof(mpu_raw_report));
#else
	mpu_raw_report.status = read_reg(fd, MPUREG_INT_STATUS);
#endif

	if (mpu_raw_report.status & BIT_RDY_INT) {
#if !defined(FAST_METHOD)
	// reading other registers somtimes clears INT_STATUS. Therefore it should be here.
	mpu_raw_report.accel_x = read_reg16(fd, MPUREG_ACCEL_XOUT_H);
	mpu_raw_report.accel_y = read_reg16(fd, MPUREG_ACCEL_YOUT_H);
	mpu_raw_report.accel_z = read_reg16(fd, MPUREG_ACCEL_ZOUT_H);	
	mpu_raw_report.temp = read_reg16(fd, MPUREG_TEMP_OUT_H);
	mpu_raw_report.gyro_x = read_reg16(fd, MPUREG_GYRO_XOUT_H);
	mpu_raw_report.gyro_y = read_reg16(fd, MPUREG_GYRO_YOUT_H);
	mpu_raw_report.gyro_z = read_reg16(fd, MPUREG_GYRO_ZOUT_H);	
#endif
		
		clock_gettime(CLOCK_MONOTONIC, &x->ts);

		x->accl[0] = mpu_raw_report.accel_x * accel_fscale;
		x->accl[1] = mpu_raw_report.accel_y * accel_fscale;
		x->accl[2] = mpu_raw_report.accel_z * accel_fscale;

		x->gyro[0] = mpu_raw_report.gyro_x * gyro_fscale;
		x->gyro[1] = mpu_raw_report.gyro_y * gyro_fscale;
		x->gyro[2] = mpu_raw_report.gyro_z * gyro_fscale;
		x->temp = mpu_raw_report.temp * temp_sensitivity +  temp_zero;

		return 1;
	}

	return 0;
}

int icm20602_close(int fd)
{
	if (fd > 0) {
		close(fd);
		return 0;
	}

	return -1;
}
