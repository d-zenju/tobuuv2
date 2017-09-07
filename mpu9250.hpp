#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <Arduino.h>
#include <Wire.h>


class MPU9250 {
private:
    /******* ADDRESS ********/
    // MPU9250 Acceleration, Gyroscope Slave Address (AD0=H: 0x69)
    uint8_t MPU9250_SLAVE_ADDRESS = 0x68;

    // MPU9250 Magnet Slave Address
    uint8_t MPU9250_MAGNET_ADDRESS = 0x0C;

    // MPU9250 Acceleration, Gyroscope Who Am I
    uint8_t MPU9250_WAI_ADDRESS = 0x75;

    // MPU9250 Acceleration, Gyroscope initialize
    uint8_t MPU9250_PWR_MGMT_1 = 0x6B;
    uint8_t MPU9250_INT_PIN_CFG = 0x37;

    // MPU9250 Acceleration Address
    uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;
    uint8_t MPU9250_ACCEL_XOUT_L = 0x3C;
    uint8_t MPU9250_ACCEL_YOUT_H = 0x3D;
    uint8_t MPU9250_ACCEL_YOUT_L = 0x3E;
    uint8_t MPU9250_ACCEL_ZOUT_H = 0x3F;
    uint8_t MPU9250_ACCEL_ZOUT_L = 0x40;

    // MPU9250 Temperature Address
    uint8_t MPU9250_TEMP_OUT_H = 0x41;
    uint8_t MPU9250_TEMP_OUT_L = 0x42;

    // MPU9250 Gyroscope Address
    uint8_t MPU9250_GYRO_XOUT_H = 0x43;
    uint8_t MPU9250_GYRO_XOUT_L = 0x44;
    uint8_t MPU9250_GYRO_YOUT_H = 0x45;
    uint8_t MPU9250_GYRO_YOUT_L = 0x46;
    uint8_t MPU9250_GYRO_ZOUT_H = 0x47;
    uint8_t MPU9250_GYRO_ZOUT_L = 0x48;

    // MPU9250 Magnet Address
    uint8_t MPU9250_MAG_ST1 = 0x02;
    uint8_t MPU9250_MAG_XOUT_L = 0x03;
    uint8_t MPU9250_MAG_XOUT_H = 0x04;
    uint8_t MPU9250_MAG_YOUT_L = 0x05;
    uint8_t MPU9250_MAG_YOUT_H = 0x06;
    uint8_t MPU9250_MAG_ZOUT_L = 0x07;
    uint8_t MPU9250_MAG_ZOUT_H = 0x08;
    uint8_t MPU9250_MAG_ST2 = 0x09;
    uint8_t MPU9250_MAG_CNTL = 0x0A;

    // MPU9250 Config
    uint8_t MPU9250_GYRO_CONFIG = 0x1B;
    uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
    /******* ADDRESS ********/

    // Address
    uint8_t _address;
    uint8_t _mag_address;
    uint8_t _who_am_i;

    // sensor datas (Accel, Gyro, Temp, Magnet)
    uint8_t agtm[22];
    
    // magnet status
    uint8_t drdy;
    uint8_t hofl;
    
    // row datas
    int16_t row_accel[3];
    int16_t row_temp;
    int16_t row_gyro[3];
    int16_t row_magnet[3];

    // Acceleration, Gyroscope Scale Select
    double _gyro_scale;
    double _accel_scale;
    double _mag_scale;

    // Calibration Magnet(min, max)
    double min_max_magnet[3][2] = {
        {-999999, -999999},
        {-999999, -999999},
        {-999999, -999999}
    };

    // read register (Accel, Gyro, Magnet)
    uint8_t readRegister(const uint8_t register_addr);
    uint8_t readMagRegister(const uint8_t register_addr);
    
    // write register (Accel, Gyro, Magnet)
    void writeRegister(const uint8_t register_addr, const uint8_t value);
    void writeMagRegister(const uint8_t register_addr, const uint8_t value);

public:
    // datas
    double accel[3];
    double temp;
    double gyro[3];
    double magnet[3];
    double yaw;
    double pitch;
    double roll;
    double ypr[3];

    // Offset (Accel, Gyro, Magnet)
    double offset_accel[3] = {0.0, 0.0, 0.0};
    double offset_gyro[3] = {0.0, 0.0, 0.0};
    double offset_magnet[3] = {0.0, 0.0, 0.0};

    MPU9250();
    void init(void);    // init
    void getSensor(void);   // get sensor value
    void calcSensor(void);  // calc sensor value
    void calcYPR(void);    // calc yaw, pitch, roll
    double readAccel(int i);    // raed accel data[0:x, 1:y, 2:z]
    double readTemp();  // read temp data
    double readGyro(int i);    // raed gyro data[0:x, 1:y, 2:z]
    double readMagnet(int i);    // raed magnet data[0:x, 1:y, 2:z]
    void calibrateMagnet(void); // calibrate magnet sensor
};


#endif