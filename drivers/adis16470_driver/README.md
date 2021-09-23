# ADIS16470 ROS Driver

This is a Driver for the I2C board from seed studios using the ADIS16470 imu module.
The board cummunicates witht the imu whrough SPI and outputs the data through I2C.
The board has 20 Bytes avalible to read, 
The first byte contains any error code from the IMU.
the next six bytes contain the gyrometer data for the three axes, xyz in words.
The next six bytes contain the accelerometer data for the three axes, xyz in words.
The next two bytes contain the temprature data as a signle word.


## vars
1. float gyro[3];
    - A float array containing the gyrodata for xyz axes in that order.
2. float accl[3];
    - A float array containing the accelerometerdata for xyz axes in that order.
3. float temp;
    - A float containg the temprature measured by the imu.

## Methods
1. int openPort(const std::string device)
    - The device on raspberry pi is typically "/dev/i2c-1" but might change to "/dev/i2c-2" or "/dev/i2c-0" depending on what devices are connected and what happens when the i2c module is connected.
    -returns -1 if failed. nothing otherwise.
2. void Adis16470::closePort()
    - closes the i2c port.
3. int update()
    - updates the accelerometer,gyro and temprature data.

## Example code
This code prints out the gyro data for the y axis while the program is running.

    ADIS16470 imu;

    imu.openPort("/dev/i2c-1");

    while(is_running){
        imu.update();

        std::cout << imu.gyro[1] << std::endl;
    }
    imu.closePort();