# dynamixel mx-106 driver
This is a ros driver for the dynamixel mx-106 servo.
This driver relies on the dynamixel_sdk rospackage to run and is made to interface better with the raspberry pi and the mx-106 servo. than the dynamixel_sdk package.

# Methods
1. initServos(int Ids[], int numberOfServos);
    - This initialises servos.
    - The Ids var contains an array of all the different Ids of the servos you want initialised and the numberOfServos var contains the number of servos in the Ids array.
2. int16_t getPresentPosition(uint8_t id);
    - Gets a value of the current position pf the servo with the specified ID, The values are scaled -4096 for -180deg and 4095 for + 180deg.
3. void getPresentPositions(uint8_t id[], int16_t positions[]);
    - This Gets all positions of the servos with the ids in the id var and puts the into the positions var in the same order.
    - NOTE: The id and positions array must contain the same amount of elements as the number of servos that were initialized.
4. void setPosition(int16_t position, uint8_t id);
    - Sets the position of the servo with the specified ID.
    - 4095 coresponds to +180 deg and -4096 = -180deg
5. 
    void setPositions(int16_t positions[], uint8_t ids[]);
    - This sets the positions of all servos with the ids specified in the ids var.
    - NOTE: The id and positions array must contain the same amount of elements as the number of servos that were initialized.
