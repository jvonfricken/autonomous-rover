
#include <string>
#include <ctime>

#include "../headers/Camera.h"

void Camera::take_picture() {
    std::string camera_string = "raspistill -o pics/";

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

    //Append time string to the end of the cameraString command
    camera_string += time_string;
    camera_string += ".jpg";
    //Make system call to the camera app
    std::system(camera_string.c_str());
}
