#pragma once
#include <functional>

class PID{
public:
    PID();
    virtual ~PID();

    void setErrorFunction(std::function<double()> function);

    double run();

    double kp{1.0};
    double ki{0.01};
    double kd{0.5};

    double dt{0.0001}; // timestep

protected:
private:
    double _lastError{0};
    double intError{0};
    std::function<double()> errorFunction;
};
