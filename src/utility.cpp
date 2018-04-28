#include <cmath>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>

#include "../headers/Camera.h"
#include "../headers/Payload.h"
#include "../headers/Imu.h"
#include "../headers/utility.h"

double get_gps_angle(double x1, double y1, double x2, double y2) {

            double xdiff = x2 - x1;
            double ydiff = y2 - y1;

            double alpha = atan2(ydiff,xdiff);
            alpha = alpha*(180.0/PI);

            //Map value to 0-360 degrees
            alpha = fmod((alpha + 360.0), 360.0); 

            return alpha;
}

double get_gps_distance(double x1, double y1, double x2, double y2) {
    double xdiff = x2 - x1;
    double ydiff = y2 - y1;

    double radius = sqrt((pow(xdiff, 2) + pow(ydiff, 2)));

    return radius;
}

std::vector<double> convert_to_cart(double latitude, double longitude) {

    std::vector<double> convertedCoords;

    //convert to radians
    latitude = latitude*(PI/180.0);
    longitude = longitude*(PI/180.0);

    double x = EARTH_RADIUS*cos(latitude)*cos(longitude);
    double y = EARTH_RADIUS*cos(latitude)*sin(longitude);

    convertedCoords.push_back(x);
    convertedCoords.push_back(y);

    return convertedCoords;
}

void log_string(std::string data_string) {

    time_t rawtime;
    //Get pointer to time struct
    struct tm * timeinfo;
    char buffer[80];

    //Pass reference to get raw time, then format to local time
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    //Format string to understandable form
    strftime(buffer,sizeof(buffer),"%d-%m-%Y-%I:%M:%S",timeinfo);

    //Create string from buffer
    std::string time_string(buffer);

    time_string += " - ";
    time_string += data_string;
    time_string += "\n";

    //Log to console
    std::cout << time_string;

    std::ofstream ofs;
    ofs.open("drone.log", std::ofstream::out | std::ofstream::app);

    //Log to file
    ofs << time_string;

    ofs.close();
}

void drop_procedure(Camera *camera, Payload *payload) {
    //Spawn threads
    std::thread camera_thread(&Camera::take_picture, camera);
    std::thread payload_thread(&Payload::drop_payload, payload);

    //Wait for tasks to finish
    payload_thread.join();
    log_string("Payload finished dropping");
    camera_thread.join();
    log_string("Picture finished taking");
}

void calibrate_imu(Imu *imu) {

    int cal = 0;

    while(cal < 3) {
        cal = imu->get_calibration_status();
        std::cout << " Cal = " << cal << std::endl;
    }

    std::cout << "IMU Calibrated. Hit enter to continue... ";

    std::string resp;
    getline (std::cin, resp);
}