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

    double output_max{1};
    double output_min{-1};

    bool antiWindup{false};

    double getIntError() const;
    double getError() const;

protected:
private:
    double _lastError{0};
    double intError{0};
    std::function<double()> errorFunction;
};
