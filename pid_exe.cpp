#include <iostream>
#include <pid.h>
#include <chrono>
#include <thread>

int main(int argc, char const *argv[])
{
    PID controller = PID();

    std::shared_ptr<double> x_desired = std::make_shared<double>(2.0);
    std::shared_ptr<double> x_current = std::make_shared<double>(0.0);
    double v_current = 0;
    double a_current = 0;

    double dt = 0.1;

    double m = 1;

    double u = 0;

    controller.setErrorFunction(
        [&, x_desired, x_current]()->double{
            const double e = *x_desired - *x_current;
            if(abs(e) < 0.0001)
                *x_desired = 0;
            return e;
        }
    );

    controller.kp = 0.4;
    controller.ki = 0.001;
    controller.kd  = 0.2;
    controller.dt = dt;

    for(int i = 0; i < 10000; ++i){
        u = controller.run();
        a_current = u/m;
        v_current += a_current * dt;
        *x_current += v_current * dt;
        std::cout << "x_current: " << *x_current << ", u: "<< u << std::endl;
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    }

    return 0;
}
