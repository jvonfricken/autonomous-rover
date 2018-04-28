#ifndef TRAVERSALENGINE_H
#define TRAVERSALENGINE_H

class TraversalEngine {
    public:
        TraversalEngine(Sonar *sonar, Propulsion *propulsion, Steering *steering, Gps *gps, Imu *imu, int drive_speed, int obstacle_threshold, int target_threshold);

        void traverse_to_target(double target_x, double target_y);

    private:
        Sonar *_sonar;
        Propulsion *_propulsion;
        Steering *_steering;
        Imu *_imu;
        Gps *_gps;

        int _drive_speed;
        int _obstacle_threshold;
        int _target_threshold;

        bool _is_there_obstacle = false;
        bool _sonar_stop = false;

        bool is_obstacle();

        void maintain_heading(double heading);

        int turnToAngle(double given_angle);

        void maintain_speed();

        int findBestAngle(std::vector<std::vector<int>> data, double target_angle);

        std::vector<std::vector<int>> parseSonarData(std::vector<int> data);

        std::vector<double> get_traversal_data(double target_x, double target_y);

        void obstacle_poller();
        
        void avoid_obstacle();
};

#endif