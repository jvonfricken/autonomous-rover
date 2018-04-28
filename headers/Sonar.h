#ifndef SONAR_H
#define SONAR_H

class Sonar {

 

    public:
        Sonar(int p);
        std::vector<int> take_measurement();
        std::vector<int> take_small_measurement();
        void reset_sonar_position();
        int read_data();

    private:
        const int PACKET_SIZE = 4;
        const int SONAR_BAUD_RATE = 9600;
        const int PWM_RANGE = 200;
        const int MIN_VALUE = 5;
        const int MAX_VALUE = 26;
        const int MEDIAN_VALUE = 15;
        const int R_ASCII_VALUE = 82;
        const int NEW_LINE_VALUE = 13;
        const int DATA_CAPTURE_TIME = 200;

        int pin;
        int fd;
        
        std::vector<int> small_sweep();
        std::vector<int> sweep();
};

#endif 