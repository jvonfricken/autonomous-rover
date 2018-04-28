#include <wiringPi.h>
#include <algorithm>
#include <thread>
#include <vector>

#include "../headers/utility.h"
#include "../headers/Gps.h"
#include "../headers/Imu.h"
#include "../headers/Sonar.h"
#include "../headers/Steering.h"
#include "../headers/Propulsion.h"
#include "../headers/TraversalEngine.h"


bool TraversalEngine::is_obstacle() {
    std::vector<int> distances = _sonar->take_small_measurement();
    for(auto distance: distances) {
        if(distance < _obstacle_threshold) {
        //There is no obstacle within threshold
            return true;
        } else {
            //There totes is an obstacle
            return false;
        }
    }
}

void TraversalEngine::maintain_heading(double heading) {
    const double BOUND_RANGE = 2.0; // In degrees

    double current_heading = _imu->read_euler()[0];
    double lower_bound = heading - BOUND_RANGE;
    double upper_bound = heading + BOUND_RANGE;

    log_string("Current heading: " + std::to_string(current_heading) + " Target Heading " + std::to_string(heading));

    if(upper_bound > 360.0) {
        //CASE #1: The lower bound has crossed 0
        upper_bound = upper_bound - 360.0;
        
        if (current_heading > upper_bound && current_heading < 180.0) {
            //The heading has crossed the upper bound in the first quadrant
            _steering->turn_left(1);
        } else if (current_heading < lower_bound && current_heading > 180.0) {
            //The heading has crossed the lower bound in the fourth quadrant
            _steering->turn_right(1);
        } else {
            //The heading is still within acceptable margins. Do nothing.
            _steering->reset_to_forward();
        }
    } else if(lower_bound < 0) {
        lower_bound = lower_bound + 360.0;

        if(current_heading > upper_bound && current_heading < 180.0) {
            //The heading has crossed the upper bound in the first quadrant
            _steering->turn_right(1);
        } else if (current_heading < lower_bound && current_heading > 180.0) {
            //The heading has crossed the lower bound in the fourth quadrant
            _steering->turn_left(1);
        } else {
            //The heading is still within acceptable margins. Do nothing.
            _steering->reset_to_forward();
        }
    } else {
        if(current_heading > upper_bound) {
            _steering->turn_left(1);
        } else if(current_heading < lower_bound) {
            _steering->turn_right(1);
        } else {
            _steering->reset_to_forward();
            //The heading is still within acceptable margins. Do nothing.
        }
    }
}

int TraversalEngine::turnToAngle(double given_angle) {
    /* This function will take an angle, and automatically turn until the 
    rover's heading is aligned with that angle. It will choose the short 
    turning angle as well */

    double current_angle = _imu->read_euler()[0];
    double angle = given_angle - current_angle;

    //Adjust for edge cases
    if(angle > 180.0) {
        angle -= 360;
    } else if(angle < -180){
        angle += 360;
    }

    double target_angle = current_angle + angle;

    double old_angle = 0.0;
    int failure_idx = 0;

    if(angle < 0) {
        _steering->turn_left(3);
        //CASE 1: Crossing 0 threshold
        if(target_angle < 0) {
            target_angle += 360.0;
            current_angle += 360.0;
            _propulsion->drive(_drive_speed);
            while(current_angle > target_angle) {
                current_angle = _imu->read_euler()[0];
                if(old_angle == current_angle) {
                    failure_idx++;
                    if(failure_idx > 50) {
                        break;
                    }
                }

                //Adjust until it crosses zero
                if(current_angle < 180.0) {
                    current_angle += 360.0;
                }

                old_angle = current_angle;
                delay(10);
            }
        } 
        //CASE 2: Not crossing 0 threshold
        else {
            _propulsion->drive(_drive_speed);
            while(current_angle > target_angle) {
                current_angle = _imu->read_euler()[0];
                if(old_angle == current_angle) {
                    failure_idx++;
                    if(failure_idx > 50) {
                        break;
                    }
                }
                old_angle = current_angle;
                delay(10);
            }
        }

    } else {
        _steering->turn_right(3);
        //CASE 3: Crossing 360 threshold
        if(target_angle > 360) {
            target_angle -= 360.0;
            current_angle -= 360.0;
            _propulsion->drive(_drive_speed);
            delay(100);
            while(current_angle < target_angle) {
                current_angle = _imu->read_euler()[0];
                if(old_angle == current_angle) {
                    failure_idx++;
                    if(failure_idx > 50) {
                        break;
                    }
                }
                //Adjust until it crosses 360
                if(current_angle > 180.0) {
                    current_angle -= 360.0;
                }

                old_angle = current_angle;
                delay(10);
            }
        }
        //CASE 4: Not crossing 360 threshold
        else {
            _propulsion->drive(_drive_speed);
            while(current_angle < target_angle) {
                current_angle = _imu->read_euler()[0];
                if(old_angle == current_angle) {
                    failure_idx++;
                    if(failure_idx > 50) {
                        break;
                    }
                }
                old_angle = current_angle;
                delay(10);
            }
            
        }
    }

    _steering->reset_to_forward();
    _propulsion->stop(1);
    return 1;
}

void TraversalEngine::maintain_speed() {
    double current_pitch = _imu->read_euler()[1];
    current_pitch = int(current_pitch);
    //Going uphill
    if(current_pitch < -2){
        log_string("Hill detected of angle: " + std::to_string(current_pitch));
        if(current_pitch < -10) {
            _propulsion->drive((_drive_speed + 30));
        } else {
            double mod = abs((current_pitch*5));
            _propulsion->drive((_drive_speed + mod));
        }
    }
    //Going downhill
    else if(current_pitch > 2) {
        log_string("Decline detected of angle: " + std::to_string(current_pitch));
        if(current_pitch > 10) {
            _propulsion->drive((_drive_speed - 30));
        } else {
            double mod = (current_pitch*5);
            _propulsion->drive((_drive_speed + mod));
        }
    }
}

int TraversalEngine::findBestAngle(std::vector<std::vector<int>> data, double target_angle) {
    std::sort(data.begin(), data.end());
    int i = 255;
    std::vector<std::vector<int>> candidate_angles;

    while(i > 0) {
        for(auto j: data) {
            //Only allow candidates that are the current distance and less than 45 degree turns
            if(j[0] == i && j[1] < abs(45)) {
                candidate_angles.push_back(j);
            }
        }

        if(candidate_angles.size() > 0) {
            int current_lowest = candidate_angles[0][1];
            for(auto j: candidate_angles) {
                if(abs(j[1]) < current_lowest) {
                    current_lowest = j[1];
                    
                }
            }
            return current_lowest;
        }

        i--;
    }

    return -90;
}

std::vector<std::vector<int>> TraversalEngine::parseSonarData(std::vector<int> data) {
    std::vector<std::vector<int>> parsedData;

    int angle = 90;

    for(auto i: data) {
        parsedData.push_back({i, angle});
        angle -= 9;
    }

    return parsedData;
}

std::vector<double> TraversalEngine::get_traversal_data(double target_x, double target_y) {
    /* Returns a vector of the radius distance and angle, respectively,
    to the target. Requires ownership of the gps */

    std::vector<double> data;

    //Check location
    std::vector<double> gps_data = _gps->take_measurement();

    //In the case of a read error, keep polling until read
    while(gps_data[0] == 0.0 && gps_data[1] == 0.0) {
        gps_data = _gps->take_measurement();
    }

    std::vector<double> card_coords = convert_to_cart(gps_data[0], gps_data[1]);
    double current_x = card_coords[0];
    double current_y = card_coords[1];

    data.push_back(get_gps_distance(current_x, current_y, target_x, target_y)*1000);
    data.push_back(get_gps_angle(current_x, current_y, target_x, target_y));

    return data;
}

void TraversalEngine::obstacle_poller() {
    /* This function will constantly poll the sonar sensor for data about potential
    obstacles in the path of the rover heading. In case that it is triggered, it will
    shut of power to propulsion and join the main thread. Requires ownership of the sonar
    and propulsion while running*/

    while(true) {
        if(is_obstacle() || _sonar_stop) {
            // In the case of a trigger, break the thread, and stop propulsion
            log_string("Sonar detected obstacle");
            _propulsion->stop(1);
            _is_there_obstacle = true;
            log_string("Sonar poller thread joined");
            break;
        }
    }
}

void TraversalEngine::avoid_obstacle() {
    /*This function is called in the case that the main process has verified a obstacle
    it will first back the drone up 4 ft. from the target, then find the angle which preserves
    The heading while still avoiding the target*/

    int clearance_distance = 48; //in
    int distance_to_target = _sonar->read_data();
    int current_heading = _imu->read_euler()[0];
    log_string("Distance to target is " + std::to_string(distance_to_target));

    // The drone is probably caught on something rather than an obstacle
    if(distance_to_target > clearance_distance) {
        //Hardcode backup for 2 seconds
        log_string("Backing up (hardcoded)");
        _propulsion->reverse(_drive_speed);
        delay(1000);
        _propulsion->stop(0);
    } 

    else {
        //Reverse the drone up to a safe clearance. 4 ft.
        log_string("Backing up " + std::to_string(clearance_distance - distance_to_target));
        _propulsion->reverse(_drive_speed);
        while(distance_to_target < clearance_distance) {
            distance_to_target = _sonar->read_data();
        }
        _propulsion->stop(0);
    }

    //Do full sweep in order to find ideal angle
    std::vector<int> sonar_data = _sonar->take_measurement();

    //Put the sonar data into a more workable format
    std::vector<std::vector<int>> parsed_sonar_data = parseSonarData(sonar_data);

    //Pick out the best angle utilizing sonar sweep data
    int angle = findBestAngle(parsed_sonar_data, current_heading);

    double target_angle = (angle + _imu->read_euler()[0]);

    if(target_angle < 0) {
        target_angle += 360;
    }

    else if(target_angle > 360) {
        target_angle -= 360;
    }

    log_string("Ideal angle to turn to: " + std::to_string(target_angle));
    //Turn to new angle
    turnToAngle(target_angle);

    //Switch the power back on
    log_string("Powered on propulsion");
    _propulsion->drive(_drive_speed);    
}

TraversalEngine::TraversalEngine(Sonar *sonar,
                                 Propulsion *propulsion,
                                 Steering *steering,
                                 Gps *gps, Imu *imu,
                                 int drive_speed,
                                 int obstacle_threshold,
                                 int target_threshold) {
    
    //Hardware
    _sonar = sonar;
    _propulsion = propulsion;
    _steering = steering;
    _gps = gps;
    _imu = imu;

    //Traversal Settings
    _drive_speed = drive_speed;
    _obstacle_threshold = obstacle_threshold;
    _target_threshold = target_threshold;
}

void TraversalEngine::traverse_to_target(double target_x, double target_y) {
    /* 
    This is the main loop for the traversal engine. It will handle the traversal
    procedures required to bring the drone within the target threshold 
    */ 

    //Set up the flags for the sonar
    _sonar_stop = false;
    _is_there_obstacle = false;

    //Get the initial geospatial dataset
    std::vector<double> traversal_data = get_traversal_data(target_x, target_y);
    double radius = traversal_data[0];
    double angle = traversal_data[1];


    //Make the inital turn to the target
    log_string("Turning to angle: " + std::to_string(angle)); 
    turnToAngle(angle);
    log_string("Inital distance to target: " + std::to_string(radius));

    //Start the sonar polling
    std::thread sonar_checker_thread(&TraversalEngine::obstacle_poller, this);
    log_string("Sonar poller thread spawned");

    //Begin traversing
    _propulsion->drive(_drive_speed);
    
    //Continue traversal while not in target
    while(radius > _target_threshold) {
        //Apply weight to heading maintainence and obstacle avoidance due to slow changing
        //nature of geospatial data -- 5:1 --
        int heading_maintainence_ratio = 5;
        for(int i = 0; i < heading_maintainence_ratio; i++) {
            //Adjust steering in order to maintain the proper heading
            maintain_heading(angle);

            //Adjusts speed depending on the pitch of the drone
            maintain_speed();

            //Check to see if the sonar has flagged a potential obstacle
            if(_is_there_obstacle) {
                //Verify sonar flag
                if(is_obstacle()) {
                    log_string("Obstacle verified, avoidance procedure initalized");
                    avoid_obstacle();
                } 
                
                else {
                    //false flag, restart traversal
                    log_string("Obstacle false flag");
                    _propulsion->drive(_drive_speed);
                }

                //Finally, restart the sonar polling, be free little thread
                std::thread sonar_checker_thread(&TraversalEngine::obstacle_poller, this);
                sonar_checker_thread.detach(); 
                log_string("Sonar poller thread spawned");
                _is_there_obstacle = false; 
            }
                
        }
        
        //Get new geospatial data
        traversal_data = get_traversal_data(target_x, target_y);
        radius = traversal_data[0];
        angle = traversal_data[1];
        log_string("Distance to Target: " + std::to_string(radius) + " Angle to Target: " + std::to_string(angle));
    }

    //Signal the sonar checking thread to join before wrapping up the traversal
    _sonar_stop = true;
    sonar_checker_thread.join(); 
}