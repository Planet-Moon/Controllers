#include <iostream>
#include <pid.h>
#include <chrono>
#include <thread>
#include <array>
#include <JsonInterface.h>

int main(int argc, char const *argv[])
{
    PID controller = PID();

    std::shared_ptr<double> x_desired = std::make_shared<double>(2.0);
    std::shared_ptr<double> x_current = std::make_shared<double>(0.0);
    double v_current = 0;
    double a_current = 0;

    double dt = 0.1;

    double m = 2;

    double u = 0;

    std::shared_ptr<double> e_last = std::make_shared<double>(0);
    std::shared_ptr<int> counter = std::make_shared<int>(0);
    std::shared_ptr<int> state = std::make_shared<int>(0);
    controller.setErrorFunction(
        [&, x_desired, x_current, e_last, dt, counter, state]()->double{
            const double e = *x_desired - *x_current;
            if(*state == 0){
                if(abs(e - *e_last)/dt < 0.001 && abs(e) < 0.0001){
                    *state = 1;
                }
            }
            else if(*state == 1){
                if(*counter > 1000){
                    *counter = 0;
                    *state = 2;
                }
                ++(*counter);
            }
            else if(*state == 2){
                // *x_desired = 0;
                *state = 0;
            }
            *e_last = e;
            return e;
        }
    );

    controller.dt = dt; // constant here - real world sampling may deviate
    controller.kp = 0.4;
    controller.ki = 0.01;
    controller.kd = 0.2;
    controller.antiWindup = true;
    controller.output_max = 0.7;
    controller.output_min = -0.7;

    const int iterations = 10000;
    std::array<double, iterations> log_x{0};
    std::array<double, iterations> log_e{0};
    std::array<double, iterations> log_u{0};
    std::array<double, iterations> log_v{0};
    std::array<double, iterations> log_t{0};

    for(int i = 0; i < iterations; ++i){
        log_x[i] = *x_current;
        log_v[i] = v_current;
        log_t[i] = i * dt;
        u = controller.run();
        log_e[i] = controller.getError();
        if(abs(v_current) < 0.001 && abs(u) < 0.3){ // Friction
            u = 0;
        }
        log_u[i] = u;
        a_current = u/m;
        v_current += a_current * dt;
        *x_current += v_current * dt;
        // std::cout << "x_current: " << *x_current << ", u: "<< u << ", intError: " << controller.getIntError() << std::endl;
        // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(dt*1000));
    }

    Json::Value log;
    for(int i = 0; i < iterations; ++i){
        log["x"][i] = log_x[i];
        log["v"][i] = log_v[i];
        log["e"][i] = log_e[i];
        log["u"][i] = log_u[i];
        log["t"][i] = log_t[i];
    }
    writeToFile("../log.json", log);

    return 0;
}
