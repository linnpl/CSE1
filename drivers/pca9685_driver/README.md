# PCA9685 PWM module drvier:

## Methods
1. int OpenPort(const std::string device, int freq)
    - Opens the i2c port for controlling the PCA9685 module.
    - The device var contains the path to the port, typically "/dev/i2c-1".
    - Freq is the frequency that the PWM signal will have.
    - Returns -1 if unsuccessful. returns nothing otherwise.
2. void closePort()
    - Closes the port.
3. void reset()
    - Resets the module
4. setPWMFreq(int freq)
    - Change the PWM frequency, freq being the frequency you want it changed to.
5. setPWM(uint8_t led, int value)
    - Sets the PWM dutycyle of a pin.
    - led is the number of the pin to change. values from 1 to 16... staring on 1 not zero!!
    - int value is the dutycycle to set the pin to. 0 being 0% and 4096 being 100%
6. int getPWM(uint8_t led)
    - gets the current pwm signal of a pin.
    - 0 being 0% 4096 being 100%.
