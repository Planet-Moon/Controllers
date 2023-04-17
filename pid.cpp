#include "pid.h"

PID::PID(){

}

PID::~PID(){

}

void PID::setErrorFunction(std::function<double()> function){
    errorFunction = function;
    _lastError = errorFunction();
}

double PID::run(){
    const double e = getError();
    const double rp = kp * e;
    const double addIntError = e * dt;
    intError += addIntError;
    if(antiWindup){
        const double abs_max_integral = abs(max_integral);
        if(intError > abs_max_integral){
            intError = abs_max_integral;
        }
        else if(intError < -1*abs_max_integral){
            intError = -1*abs_max_integral;
        }
    }
    const double ri = ki * intError;
    const double e_dot = (e - _lastError)/dt;
    const double rd = kd * e_dot;
    double output = rp + ri + rd;
    if(antiWindup){
        if(output > output_max){
            output = output_max;
        }
        else if(output < output_min){
            output = output_min;
        }
    }

    _lastError = e;
    return output;
}

double PID::getIntError() const {
    return intError;
}

double PID::getError() const {
    if(!errorFunction)
        return 0.0;
    return errorFunction();
}
