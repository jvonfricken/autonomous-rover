#include <algorithm>
#include <iterator>
#include <fstream>
#include <istream>
#include <vector>
#include <sstream>

#include "../headers/Parser.h"
#include "../headers/Gps.h"
#include "../headers/utility.h"
#include "../headers/planner.h"

std::vector<std::vector<double>> traveling_drone_problem(std::vector<std::vector<double>> gps_coords, std::vector<double> start_coord) {
    /*
    This function takes a vector of vector of doubles. it will then sort them according to the total
    shortest path between them. This is a brute force solution to the Traveling Salesmen Problem.
    */

    std::vector<std::vector<double>> xy_coords;
    double total_distance = 0.0;
    double previous_shortest = 0.0;

    for(auto i: gps_coords) {
        std::vector<double> xy_vector_data = convert_to_cart(i[0], i[1]);
        std::vector<double> xy_vector;
        xy_vector.push_back(xy_vector_data[0]);
        xy_vector.push_back(xy_vector_data[1]);
        xy_vector.push_back(i[0]);
        xy_vector.push_back(i[1]);
        xy_coords.push_back(xy_vector);
    }

    std::vector<std::vector<double>> new_list = xy_coords;
    std::vector<double> previous_coords = convert_to_cart(start_coord[0], start_coord[1]);
    std::vector<std::vector<double>> shortest_route = xy_coords;

    for(int k = 0; k < xy_coords.size(); k++) {
            previous_shortest += get_gps_distance(previous_coords[0], previous_coords[1], xy_coords[k][0], xy_coords[k][1]);
            previous_coords = {xy_coords[k][0], xy_coords[k][1]};
        }

    std::sort(xy_coords.begin(), xy_coords.end());
    do {
        previous_coords = convert_to_cart(start_coord[0], start_coord[1]);
        for(int k = 0; k < xy_coords.size(); k++) {
            total_distance += get_gps_distance(previous_coords[0], previous_coords[1], xy_coords[k][0], xy_coords[k][1]);
            previous_coords = {xy_coords[k][0], xy_coords[k][1]};
                    
        }

        if(total_distance < previous_shortest) {
            previous_shortest = total_distance;
            shortest_route = xy_coords;
        }
        total_distance = 0;
        
    } while (std::next_permutation(xy_coords.begin(), xy_coords.end()));

    return shortest_route;
}

std::vector<std::vector<double>> get_traversal_locations(Gps *gps, Parser parser) {
    //Read the intended coordinates from file 
    std::vector<std::vector<double>> gps_coords = collect_gps_coordinates(parser);


    // Get the starting location of the drone
    std::vector<double> current_coords = {0.0, 0.0};
    while(current_coords[0] == 0.0 && current_coords[1] == 0.0) {
        current_coords = gps->take_measurement();
    }
    log_string("Starting Latitude: " + std::to_string(current_coords[0]) + " Starting Longitude: " + std::to_string(current_coords[1]));

    //Sort gps coordinates for minimum total distance
    std::vector<std::vector<double>> gps_coords_sorted = traveling_drone_problem(gps_coords, current_coords);

    return gps_coords_sorted;
}


std::vector<std::vector<double>> collect_gps_coordinates(Parser parser) {

    std::vector<std::vector<double>> gps_coords;

    std::vector<std::string> locations = parser.get_key_values("location");

    for(auto location: locations) {
        std::istringstream buf(location);
        std::istream_iterator<std::string> beg(buf), end;

        std::vector<std::string> tokens(beg, end); // done!

        double latitude = atof (tokens[0].c_str());
        double longitude = atof (tokens[1].c_str());
        gps_coords.push_back({latitude, longitude});
    }

    return gps_coords;
}
