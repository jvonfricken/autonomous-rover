#ifndef UTILITY_H
#define UTILITY_H

#include "../headers/Camera.h"
#include "../headers/Payload.h"
#include "../headers/Imu.h"

double const EARTH_RADIUS = 6371.0; //km
double const PI = 3.14159265;

double get_gps_angle(double x1, double y1, double x2, double y2);

double get_gps_distance(double x1, double y1, double x2, double y2);

std::vector<double> convert_to_cart(double latitude, double longitude);

void log_string(std::string data_string);

void drop_procedure(Camera *camera, Payload *payload);

void calibrate_imu(Imu *imu);

#endif