#ifndef PROPULSION_H
#define PROPULSION_H

class Propulsion {

	public:
		Propulsion(int r1Pin, int r2Pin, int tPin);
		void drive(int speed);
		void reverse(int speed);
        void stop(int direction);

    private:
        int relay1Pin;
        int relay2Pin;
        int throttlePin;
};

#endif