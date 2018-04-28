#include <wiringPi.h>
#include <string>
#include <vector>

#include "../headers/Camera.h"
#include "../headers/Sonar.h"
#include "../headers/Steering.h"
#include "../headers/Propulsion.h"
#include "../headers/Payload.h"
#include "../headers/Gps.h"
#include "../headers/Imu.h"
#include "../headers/TraversalEngine.h"
#include "../headers/utility.h"
#include "../headers/planner.h"
#include "../headers/Parser.h"

int main(int argc, char **argv[]) {
    log_string("=================== BEGIN RUN ===================\n\n\n");

    
    Parser parser("doc/config.txt");
    
    //Extract pin layload information from drone config
    parser.get_key_value("imu_pin");
    
    int imu_pin = stoi(parser.get_key_value("imu_pin"));
    int steering_pin = stoi(parser.get_key_value("steering_pin"));
    int propulsion_pin_1 = stoi(parser.get_key_value("propulsion_pin_1"));
    int propulsion_pin_2 = stoi(parser.get_key_value("propulsion_pin_2"));
    int propulsion_pin_3 = stoi(parser.get_key_value("propulsion_pin_3"));
    int payload_pin = stoi(parser.get_key_value("payload_pin"));
    int sonar_pin = stoi(parser.get_key_value("sonar_pin"));

    //Extract operational params from drone config
    int drive_speed = stoi(parser.get_key_value("drive_speed"));
    int obstacle_threshold = stoi(parser.get_key_value("obstacle_threshold"));
    int target_threshold = stoi(parser.get_key_value("target_threshold"));

    // Initialize hardware
    wiringPiSetup();

    Imu imu(imu_pin);
    imu.begin();
    Steering steering(steering_pin);
    Payload payload(payload_pin);
    Sonar sonar(sonar_pin);
    Propulsion propulsion(propulsion_pin_1, propulsion_pin_2, propulsion_pin_3);
    Camera camera;
    Gps gps;

    log_string("=================== HARDWARE INITIALIZED ===================");

    //Block process until the IMU has been calibrated
    calibrate_imu(&imu);
    log_string("=================== IMU CALIBRATED ===================");

    //Get traversal location data
    std::vector<std::vector<double>> locations = get_traversal_locations(&gps, parser);

    //Spawn the traversal engine and give ownership of the hardware
    TraversalEngine traversal_engine(&sonar, &propulsion, &steering, &gps, &imu, drive_speed, obstacle_threshold, target_threshold);
   
    //Iterate over each of the locations
    for(auto location: locations) {
        log_string("=================== TRAVERSING TO POINT: " + std::to_string(location[2]) + ", " + std::to_string(location[3]) + " ===================");
        traversal_engine.traverse_to_target(location[0], location[1]);
        log_string("=================== DROPPING PAYLOAD AT POINT: " + std::to_string(location[2]) + ", " + std::to_string(location[3]) + " ===================");
        drop_procedure(&camera, &payload);
    }

    return 0;
}
