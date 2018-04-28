#ifndef IMU_H
#define IMU_H

class Imu {

    public:
        Imu(int rst);

        bool begin();

        int get_calibration_status();

        std::vector<double> read_euler();

        std::vector<double> read_magnetometer();

        std::vector<double> read_accelerometer();

    private:

        //Operation mode registers
        const char OPERATION_MODE_NDOF = 0X0C;
        const char OPERATION_MODE_CONFIG = 0X00;
        const char OPERATION_MODE_COMPASS = 0X09;

        //Page id register definition
        const char BNO055_PAGE_ID_ADDR = 0X07;

        //Op mode register
        const char BNO055_OPR_MODE_ADDR = 0X3D; 

        //System and power registers
        const char BNO055_PWR_MODE_ADDR = 0X3E;
        const char POWER_MODE_NORMAL = 0X00;
        const char BNO055_SYS_TRIGGER_ADDR = 0X3F;

        //Euler Register
        const char BNO055_EULER_H_LSB_ADDR = 0X1A;

        //Mag Register
        const char BNO055_MAG_DATA_X_LSB_ADDR = 0X0E;

        const char BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08;
        const char BNO055_CALIB_STAT_ADDR = 0X35;

        int _serial_num;
        int _rst;

        void write_byte(char address, char value);

        int serial_send(char* command, int length);

        void set_mode(char mode);

        void config_mode();

        void operation_mode();

        std::vector<char> read_bytes(char address, int length);

        std::vector<double> read_vector(char address);

};

#endif