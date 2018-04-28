#include <wiringSerial.h>
#include <wiringPi.h>
#include <vector>
#include <unistd.h>  

#include "../headers/Imu.h"

void Imu::write_byte(char address, char value) {
    char command [5];

    command[0] = 0xAA; //Start byte
    command[1] = 0x00; //Write
    command[2] = address & 0xFF;
    command[3] = 1;     //Length (1 byte)
    command[4] = value & 0xFF;

    //Send off the command to the serial port
    serial_send(command, 5);
}

int Imu::serial_send(char* command, int length) {
    /*
    This function is the main entry point for data ingress and egress
    to the sensor. It will take a byte array, prepare the buffer, and 
    then send it along to the sensor, and verify the response bytes do 
    not indicate errors.
    */
    char resp [2];

    // Loop 5 times, this will help in the case that the sensor packet 
    // fails 
    for(int i = 0; i < 5; i++) {
        //Flush the buffer in case some pesky bytes are left
        serialFlush(_serial_num);

        //Write to the sensor, and read the response bytes
        write(_serial_num, command, length);
        resp[0] = serialGetchar(_serial_num);
        resp[1] = serialGetchar(_serial_num);
        
        if(!(resp[0] == 0xEE and resp[1] == 0x07)) {
            //Verify that the bytes do no indicate an error
            return resp[1];
        }
    }
}

void Imu::set_mode(char mode) {
    //Set the operation mode for the BNO055 Sensor
    write_byte(BNO055_OPR_MODE_ADDR, mode & 0xFF);
    //Delay for 30 milliseconds (datsheet recommends 19ms, but a little more
    //can't hurt and the kernel is going to spend some unknown amount of time
    //too).
    delay(30);
}

void Imu::config_mode() {
    set_mode(OPERATION_MODE_CONFIG);
}

void Imu::operation_mode() {
    set_mode(OPERATION_MODE_NDOF);
}

std::vector<char> Imu::read_bytes(char address, int length) {
    char command [4];
    command[0] = 0xAA;  //Start byte
    command[1] = 0x01; //Read
    command[2] = address & 0xFF;
    command[3] = length & 0xFF;
    int data_length = serial_send(command, 4);

    std::vector<char> data;

    for(int i = 0; i < data_length; i++) {
        char val = serialGetchar(_serial_num);
        data.push_back(val);
    }

    return data;
}

std::vector<double> Imu::read_vector(char address) {
    /*
    This function will sensor a vector from the sensor and then
    translate it into SI standard units.
    */

    // Read raw data from sensor
    std::vector<char> data = read_bytes(address, 6);
    std::vector<double> result = {0, 0, 0};

    //Bitwise operations to translate char bytes into degrees
    for(int i = 0; i < 3; i++) {
        result[i] = ((data[i*2+1] << 8) | data[i*2]) & 0xFFFF;
        if (result[i] > 32767) {
            result[i] -= 65536;
        }

        result[i] = result[i]/16.0;
            
    }

    return result;
}

Imu::Imu(int rst) {
    _rst = rst;
    pinMode(rst, OUTPUT);
    digitalWrite(rst, HIGH);
    delay(650);

    _serial_num = serialOpen("/dev/ttyUSB0", 115200);
}

bool Imu::begin() {
    /*
    This function sets up everything related to the BNO055 Sensor. 
    It toggles the reset pin, as well as sending some mock serial
    bytes to clear out the internal sensor buffer. It will then enter
    operation mode. This function MUST be called before any other 
    class functions are called.
    */

    //clear everything
    write_byte(BNO055_PAGE_ID_ADDR, 0);
    config_mode();

    //toggle reset pin
    digitalWrite(_rst, LOW);
    delay(10);
    digitalWrite(_rst, HIGH);

    //Recommended time for sensor to reset
    delay(650);

    write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    write_byte(BNO055_SYS_TRIGGER_ADDR, 0x0);
    operation_mode();
    return true;
}

int Imu::get_calibration_status() {
        /*Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
            - System, 3=fully calibrated, 0=not calibrated
            - Gyroscope, 3=fully calibrated, 0=not calibrated
            - Accelerometer, 3=fully calibrated, 0=not calibrated
            - Magnetometer, 3=fully calibrated, 0=not calibrated
        */

        char cal_status = read_bytes(BNO055_CALIB_STAT_ADDR, 1)[0];

        int mag = cal_status & 0x03;
        return mag;
}

std::vector<double> Imu::read_euler() {
    /*
    Return the current absolute orientation as a vector of heading, roll,
    and pitch euler angles in degrees.
    */
    std::vector<double> raw_data = read_vector(BNO055_EULER_H_LSB_ADDR);
    for(auto i: raw_data) {
        i = i/16.0;
    }


    raw_data[0] += 250.0;
    if(raw_data[0] > 360.0) {
        raw_data[0] -= 360;
    }

    raw_data[1] += 4.0;

    return raw_data;
}

std::vector<double> Imu::read_magnetometer() {
    /*
    Return the current magnetometer reading as a vector of doubles
    in units of micro-Teslas
    */
    std::vector<double> raw_data = read_vector(BNO055_MAG_DATA_X_LSB_ADDR);
    for(auto i: raw_data) {
        i = i/16.0;
    }

    return raw_data;
}

std::vector<double> Imu::read_accelerometer() {
    /*
    Return the current accelerometer reading as a vector of double values
    in meters/second^2.
    */
    std::vector<double> raw_data = read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR);
    for(auto i: raw_data) {
        i = i/100.0;
    }



    return raw_data;
} 