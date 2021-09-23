
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>     
#include <fcntl.h>
#include <syslog.h>		
#include <inttypes.h>
#include <errno.h>
#include <math.h>

#include "pca9685_driver/pca9685.h"


/**
 * @brief Constructor
 */
PCA9685::PCA9685()
  : fd_(-1)
{
}

/**
 * @brief Open device
 * @param device Device file name (/dev/i2c-1)
 * @param freq PWM frequency
 * @retval 0 Success
 * @retval -1 Failure
 */
int PCA9685::OpenPort(const std::string device, int freq)
{
  fd_ = open(device.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    perror("openPort");
    return -1;
  }
  ioctl(fd_,I2C_SLAVE, 0x40);

  setPWMFreq(freq);

  return 0;
}

/**
 * @brief Closes port.. duh
 */
void PCA9685::closePort()
{
  close(fd_);
}

/**
 * @brief resets the PCA module
 */
void PCA9685::reset() {

	unsigned char buff[2];
	buff[0] = MODE1;
	buff[1] = 0x00;
	write(fd_, buff,2); //Normal mode
	buff[0] = MODE2;
	buff[1] = 0x04;
	write(fd_,buff,2); //totem pole

}

/**
 * @brief sets the frequency of the PWM signal.
 * @param freq desired PWM frequency in Hz.
 */
void PCA9685::setPWMFreq(int freq) {
	uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
	unsigned char buff[2];
	buff[0] = MODE1;
	buff[1] = 0x10;
	write(fd_, buff, 2); //sleep
	buff[0] = PRE_SCALE;
	buff[1] = prescale_val;
	write(fd_, buff, 2); // multiplyer for PWM frequency
	buff[0] = MODE1;
	buff[1] = 0x80;
	write(fd_, buff, 2); //restart
	buff[0] = MODE2;
	buff[1] = 0x04;
	write(fd_, buff, 2); //totem pole (default)
}


/**
 @brief sets the PWM signal on time.
 @param led PWM channel to set
 @param value value to set the PWM signal to, 0 being 0% on and 4096 being 100% on.
 */
void PCA9685::setPWM(uint8_t led, int value) {
	setPWM(led, 0, value);
}

/**
 @brief sets pwm signal while allowing you to change the time it turns off.
 @param led PWM channel to set
 @param on_value 0-4095 time to turn on the PWM signal
 @param off_value 0-4095 time to turn off the PWM signal
 */

void PCA9685::setPWM(uint8_t led, int on_value, int off_value) {
	unsigned char buff[2];
	buff[0] = LED0_ON_L + LED_MULTIPLYER * (led - 1);
	buff[1] = on_value & 0xFF;
	write(fd_, buff, 2);
	buff[0] = LED0_ON_H + LED_MULTIPLYER * (led - 1);
	buff[1] = on_value >> 8;
	write(fd_, buff, 2);
	buff[0] = LED0_OFF_L + LED_MULTIPLYER * (led - 1);
	buff[1] = off_value & 0xFF;
	write(fd_, buff, 2);
	buff[0] = LED0_OFF_H + LED_MULTIPLYER * (led - 1);
	buff[1] = off_value >> 8;
	write(fd_, buff, 2);
}

/**
 @brief Get current PWM value on specified pin.
 @param led specify pin 1-16
 */
int PCA9685::getPWM(uint8_t led){
	int ledval = 0;
	unsigned char buff[1];
	buff[0] = LED0_OFF_H + LED_MULTIPLYER * (led-1);
	read(fd_, buff, 1);
	ledval = buff[0];
	ledval = ledval & 0xf;
	ledval <<= 8;
	buff[0] = LED0_OFF_L + LED_MULTIPLYER * (led-1);
	ledval += buff[0];
	return ledval;
}
