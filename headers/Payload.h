#ifndef PAYLOAD_H
#define PAYLOAD_H

class Payload {

    public:
        Payload(int p);
        void drop_payload();
    
    private:
        int pin;
        const int PWM_RANGE = 200;
        const int START_NUM = 5;
        const int END_NUM = 20;

        void set_back_position();
        void set_forward_position();
};

#endif