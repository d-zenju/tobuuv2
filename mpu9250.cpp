#include "mpu9250.hpp"

MPU9250::MPU9250(void) {
    Wire.begin();
    _address = MPU9250_SLAVE_ADDRESS;
    _mag_address = MPU9250_MAGNET_ADDRESS;
}


uint8_t MPU9250::readRegister(const uint8_t register_addr) {
    uint8_t data = 0;
  
    Wire.beginTransmission(_address);
    Wire.write(register_addr);
    Wire.endTransmission();

    Wire.requestFrom((int)_address, 1);

    while(Wire.available()) {
        data = Wire.read();
    }

    return data;
}
  
  
void MPU9250::writeRegister(const uint8_t register_addr, const uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(register_addr);
    Wire.write(value);
    Wire.endTransmission();
}
  
  
uint8_t MPU9250::readMagRegister(const uint8_t register_addr) {
    uint8_t data = 0;
    
    Wire.beginTransmission(_mag_address);
    Wire.write(register_addr);
    Wire.endTransmission();
  
    Wire.requestFrom((int)_mag_address, 1);
  
    while(Wire.available()) {
        data = Wire.read();
    }
  
    return data;
}
  
  
void MPU9250::writeMagRegister(const uint8_t register_addr, const uint8_t value) {
    Wire.beginTransmission(_mag_address);
    Wire.write(register_addr);
    Wire.write(value);
    Wire.endTransmission();
}


void MPU9250::init(void) {
    _who_am_i = readRegister(MPU9250_WAI_ADDRESS);
    if (_who_am_i != 0x71) exit(0);
  
    // Acceleration, Gyroscope Sensing
    writeRegister(MPU9250_PWR_MGMT_1, 0x00);
    writeRegister(MPU9250_INT_PIN_CFG, 0x02);
    writeMagRegister(MPU9250_MAG_CNTL, 0x06);
  
    // get Acceleration, Gyroscope Config
    uint8_t gyroConfig = readRegister(MPU9250_GYRO_CONFIG);
    uint8_t accelConfig = readRegister(MPU9250_ACCEL_CONFIG);
    uint8_t magnetConfig = readMagRegister(MPU9250_MAG_CNTL);
  
    gyroConfig = gyroConfig << 3;
    uint8_t gc = gyroConfig >> 6;
    accelConfig = accelConfig << 3;
    uint8_t ac = accelConfig >> 6;
    uint8_t mc = magnetConfig >> 4;
  
    switch(gc) {
        case 0b00:  // gyro_scale ±250(deg/sec)
            _gyro_scale = 250.0;
            break;
        case 0b01:  // gyro_scale ±500(deg/sec)
            _gyro_scale = 500.0;
            break;
        case 0b10:  // gyro_scale ±1000(deg/sec)
            _gyro_scale = 1000.0;
            break;
        case 0b11:  // gyro_scale ±2000(deg/sec)
            _gyro_scale = 2000.0;
            break;
        default:
            break;
    }
    
    switch(ac) {
        case 0b00:  // accel_scale ±2(g)
            _accel_scale = 2.0;
            break;
        case 0b01:  // accel_scale ±4(g)
            _accel_scale = 4.0;
            break;
        case 0b10:  // accel_scale ±8(g)
            _accel_scale = 8.0;
            break;
        case 0b11:  // accel_scale ±16(g)
            _accel_scale = 16.0;
            break;
        default:
            break;
    }
    
    switch(mc) {
        case 0b0000:  // magnet_scale 14bit
            _mag_scale = 8190.0;
            break;
        case 0b0001:  // magnet_scale 16bit
            _mag_scale = 32760.0;
            break;
        default:
            break;
    }
}


void MPU9250::getSensor(void) {
    // Accel, Gyro, Temp
    Wire.beginTransmission(_address);
    Wire.write(MPU9250_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom((int)_address, 14);
    for (int i = 0; i < 14; i++)
        agtm[i] = Wire.read();
    
    // Magnet
    Wire.beginTransmission(_mag_address);
    Wire.write(MPU9250_MAG_ST1);
    Wire.endTransmission();
    Wire.requestFrom((int)_mag_address, 8);
    for (int i = 14; i < 22; i++)
        agtm[i] = Wire.read();

    row_accel[0] = (((int16_t)agtm[0]) << 8) | agtm[1];
    row_accel[1] = (((int16_t)agtm[2]) << 8) | agtm[3];
    row_accel[2] = (((int16_t)agtm[4]) << 8) | agtm[5];
    row_temp = (((int16_t)agtm[6]) << 8) | agtm[7];
    row_gyro[0] = (((int16_t)agtm[0]) << 8) | agtm[1];
    row_gyro[1] = (((int16_t)agtm[2]) << 8) | agtm[3];
    row_gyro[2] = (((int16_t)agtm[4]) << 8) | agtm[5];
    drdy = agtm[14];
    row_magnet[0] = (((int16_t)agtm[16]) << 8) | agtm[15];
    row_magnet[1] = (((int16_t)agtm[18]) << 8) | agtm[17];
    row_magnet[2] = (((int16_t)agtm[20]) << 8) | agtm[19];
    hofl = agtm[21];
}


void MPU9250::calcSensor(void) {
    for (int i = 0; i < 3; i++) {
        accel[i] = (double)row_accel[i] * _accel_scale / 32768.0 - offset_accel[i];
        gyro[i] = (double)row_gyro[i] * _gyro_scale / 32768.0 - offset_gyro[i];
        magnet[i] = (double)row_magnet[i] * 4912.0 - offset_magnet[i];
    }
    temp = (double)row_temp / 333.87 + 21.0;
}


void MPU9250::calcYPR(void) {
    roll = atan(accel[1] / sqrt(accel[0] * accel[0] + accel[2] * accel[2]));
    pitch = atan(-accel[0] / accel[2]);
    double hx = magnet[0] * cos(roll) + magnet[2] * sin(roll);
    double hy = magnet[1] * cos(pitch) + sin(pitch) * (magnet[0] * sin(roll) - magnet[2] * cos(roll));
    yaw = atan2(hx, hy);
    if (yaw < 0)
        yaw += 2 * PI;
    ypr[0] = yaw;
    ypr[1] = pitch;
    ypr[2] = roll;
}


double MPU9250::readAccel(int i) {
    return accel[i];
}


double MPU9250::readTemp() {
    return temp;
}


double MPU9250::readGyro(int i) {
    return gyro[i];
}


double MPU9250::readMagnet(int i) {
    return magnet[i];
}