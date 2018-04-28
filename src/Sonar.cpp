#include <vector>
#include <string>

#include <wiringSerial.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "../headers/Sonar.h"

Sonar::Sonar(int p) {
    //Initial config for GPIO pins
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    softPwmCreate(pin, MEDIAN_VALUE, PWM_RANGE);
    fd = serialOpen("/dev/ttyUSB2", SONAR_BAUD_RATE);
}

std::vector<int> Sonar::take_measurement() {

    //Perform a sweep with the servo and sonar
    std::vector<int> data = sweep();

    //Reset ther sonar back to the forward position
    reset_sonar_position();

    return data;
}

std::vector<int> Sonar::take_small_measurement() {
    //Perform a sweep with the servo and sonar
    std::vector<int> data = small_sweep();

    //Reset ther sonar back to the forward position
    reset_sonar_position();

    return data;
}

void Sonar::reset_sonar_position() {
    softPwmWrite(pin, MEDIAN_VALUE);
    delay(100);
}

int Sonar::read_data() {
    //Open serial port for the USB input from sonar
    
    int dataLength = serialDataAvail(fd);
    
    //Clear buffer
    for (int i = 0; i < dataLength; i++) {
        serialGetchar(fd);
    }
    //Wait for a second to allow it to gather data
    delay(200);

    //Get the length of the data
    dataLength = serialDataAvail(fd);

    char dataArray [dataLength];
    
    //extract data from buffer
    for (int i = 0; i < dataLength; i++) {
        dataArray[i] = serialGetchar(fd);
    }

    char datum;

    //prepare vector for formatted data packets
    std::vector<char> dataPackets;

    //parse and format data
    for(int i = 0; i < dataLength; i++) {
        datum = dataArray[i];

        //Start filling the packets
        if(datum == NEW_LINE_VALUE) {
            //Do Nothing
        } else if(datum == R_ASCII_VALUE ) {
            char comma = char(44);
            dataPackets.push_back(comma);
        } else {
            dataPackets.push_back(char(datum));
        }
    }



        int j = 0;
        std::string packet;
        std::vector<int> packets;
        int packetInt;
        int val;
        int mutli = 1;

        for (auto i: dataPackets) {
            
            if(int(i) == 44) {
                packetInt = atoi(packet.c_str());
                packet = "";
                packets.push_back(packetInt);
            } else {
                packet += i;
            }
        }



        int k = 0;
        int totValue = 0;

        for(auto i: packets) {
            totValue += i;
            k++;
        }
        int value = totValue/k;
        return value;
}

std::vector<int> Sonar::sweep() {
    std::vector<int> data;

    for(int i = MIN_VALUE; i < MAX_VALUE; i++) {
        softPwmWrite(pin, i);
        //Take a data measurement
        data.push_back(read_data());
    }

    return data;
}

std::vector<int> Sonar::small_sweep() {
    std::vector<int> data;

    //Take a small measurement of +- 10 degrees, amounting to three datapoints
    for(int i = (MEDIAN_VALUE - 2); i < (MEDIAN_VALUE + 1); i++) {
        softPwmWrite(pin, i);
        //Take a data measurement
        data.push_back(read_data());
    }

    return data;
}
