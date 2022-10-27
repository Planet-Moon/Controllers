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
    if(!errorFunction)
        return 0.0;
    const double e = errorFunction();
    const double rp = kp * e;
    const double ri = ki * intError;
    const double e_dot = (e - _lastError)/dt;
    const double rd = kd * e_dot;
    const double result = rp + ri + rd;
    const double addIntError = e * dt;
    intError += addIntError;
    _lastError = e;
    return result;
}
