#ifndef STEERING
#define STEERING

class Steering {
    public:
        Steering(int p);
        void turn_left(int sharpness);
        void turn_right(int sharpness);
        void reset_to_forward();

    private:
        int pin;
    	struct steering_values
    	{
	    	const int MAX_LEFT_STEERING_NUM = 20;
	        const int MEDIUM_LEFT_STEERING_NUM = 13;
	        const int SHALLOW_LEFT_STEERING_NUM = 13;
	        const int MAX_RIGHT_STEERING_NUM = 4;
	        const int MEDIUM_RIGHT_STEERING_NUM = 6;
	        const int SHALLOW_RIGHT_STEERING_NUM = 11;
	        const int FORWARD_STEERING_NUM = 12;
    	};
    
};

#endif