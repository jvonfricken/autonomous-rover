#ifndef PLANNER_H
#define PLANNER_H

#include "../headers/Parser.h"
#include "../headers/Gps.h"

std::vector<std::vector<double>> traveling_drone_problem(std::vector<std::vector<double>> gps_coords, std::vector<double> start_coord);

std::vector<std::vector<double>> get_traversal_locations(Gps *gps, Parser parser);

std::vector<std::vector<double>> collect_gps_coordinates(Parser parser);

#endif