#include <vector>
#include <iostream>
#include <libgpsmm.h>

#include "../headers/Gps.h"

std::vector<double> Gps::take_measurement() {

    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
        std::cout << "GPSD is not running" << std::endl;
        return {0.0,0.0};
    }

    struct gps_data_t *gpsd_data;

    
    if ((gpsd_data = gps_rec.read()) == NULL) {
    return {0.0,0.0};
    } else {
    while (((gpsd_data = gps_rec.read()) == NULL) ||
            (gpsd_data->fix.mode < MODE_2D)) {
        // Do nothing until fix
    }
    double latitude  = gpsd_data->fix.latitude;
    double longitude = gpsd_data->fix.longitude;

    return {latitude, longitude};
    }
}
