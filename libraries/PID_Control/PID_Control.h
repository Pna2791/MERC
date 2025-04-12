#ifndef PID_CONTROL_H
#define PID_CONTROL_H


#define FREQ    25


class PIController {
public:
    PIController(float kp, float ki, float max_output) {
        this->kp = kp;
        this->ki = ki;
        integral = 0;
        outputMin = -max_output;
        outputMax = max_output;
    }

    void reset(){
        integral = 0;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        integral += error;
        integral = constrain(integral, -255, 255);
        float output = (kp * error) + (ki * integral);

        return constrain(output, outputMin, outputMax);
    }

private:
    float kp, ki;
    float integral;
    float outputMin, outputMax;
};


class PDController {
public:
    PDController(float kp, float kd, float max_output) {
        this->kp = kp;
        this->kd = kd;
        prevError = 0;
        outputMin = -max_output;
        outputMax = max_output;
    }

    void reset(){
        prevError = 0;
    }

    void set_P_D(float p, float d){
        kp = p;
        kd = d;
    }
    void set_P(float p){
        kp = p;
    }
    void set_D(float d){
        kd = d;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        float derivative = (error - prevError) * FREQ;
        float output = (kp * error) + (kd * derivative);

        prevError = error;
        return constrain(output, outputMin, outputMax);
    }

private:
    float kp, kd;
    float prevError;
    float outputMin, outputMax;
};


class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_output) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        integral = 0;
        prevError = 0;
        outputMin = -max_output;
        outputMax = max_output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

    void set_P_I_D(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void set_P(float p) {
        kp = p;
    }

    void set_I(float i) {
        ki = i;
    }

    void set_D(float d) {
        kd = d;
    }

    float compute(float setpoint, float measured) {
        float error = setpoint - measured;
        integral += error / FREQ; // dt = 1/FREQ
        integral = constrain(integral, outputMin/2/ki, outputMax/2/ki);
        float derivative = (error - prevError) * FREQ;

        float output = (kp * error) + (ki * integral) + (kd * derivative);
        prevError = error;

        return constrain(output, outputMin, outputMax);
    }

private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float outputMin, outputMax;
};


#endif