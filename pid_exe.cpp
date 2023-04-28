#include <iostream>
#include <pid.h>
#include <chrono>
#include <thread>
#include <array>
#include <JsonInterface.h>

void desired_x(){
    std::cout << "desired_x" << std::endl;
    PID controller = PID();

    std::shared_ptr<double> x_desired = std::make_shared<double>(2.0);
    std::shared_ptr<double> x_current = std::make_shared<double>(0.0);
    double v_current = -0.5;
    double a_current = 0;

    const double dt = 0.1;

    const double m = 2;

    double u = 0;

    double f_friction = 0;

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
                *x_desired = -2;
                *state = 0;
            }
            *e_last = e;
            return e;
        }
    );

    controller.dt = dt; // constant here - real world sampling may deviate
    controller.kp = 0.5;
    controller.ki = controller.kp * 0.001;
    controller.kd = controller.kp * 0.5;
    controller.antiWindup = true;
    controller.output_max = 1;
    controller.output_min = -1;

    const int iterations = 10000;
    std::array<double, iterations> log_x{0};
    std::array<double, iterations> log_e{0};
    std::array<double, iterations> log_u{0};
    std::array<double, iterations> log_v{0};
    std::array<double, iterations> log_t{0};
    std::array<double, iterations> log_fric{0};

    for(int i = 0; i < iterations; ++i){
        log_x[i] = *x_current;
        log_v[i] = v_current;
        log_t[i] = i * dt;
        u = controller.run();
        log_e[i] = controller.getError();
        // if(abs(v_current) < 0.001 && abs(u) < 0.3){ // Friction
        //     u = 0;
        // }
        log_u[i] = u;
        f_friction = 0.3 * v_current * v_current * m; // Friction Force
        if(v_current > 0){
            f_friction *= -1;
        }
        log_fric[i] = f_friction;
        a_current = (u+f_friction)/m;
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
        log["fric"][i] = log_fric[i];
    }
    writeToFile("../log.json", log);
}

void desired_v(){
    std::cout << "desired_v" << std::endl;
    PID controller = PID();

    std::shared_ptr<double> v_desired = std::make_shared<double>(2.0);
    std::shared_ptr<double> v_current = std::make_shared<double>(1.8);
    double x_current = 0;
    double a_current = 0.5;

    const double dt = 0.1;

    const double m = 2;

    double u = 0;

    double f_friction = 0;

    std::shared_ptr<double> e_last = std::make_shared<double>(0);
    std::shared_ptr<int> counter = std::make_shared<int>(0);
    std::shared_ptr<int> state = std::make_shared<int>(0);
    controller.setErrorFunction(
        [&, v_desired, v_current, e_last, dt, counter, state]()->double{
            const double e = *v_desired - *v_current;
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
                *v_desired = -2;
                *state = 0;
            }
            *e_last = e;
            return e;
        }
    );

    controller.dt = dt; // constant here - real world sampling may deviate
    controller.kp = 1.5;
    controller.ki = controller.kp * 0.02;
    controller.kd = controller.kp * 0.5;
    controller.antiWindup = false;
    controller.output_max = 1;
    controller.output_min = -1;
    controller.bounded_output = false;

    const int iterations = 10000;
    std::array<double, iterations> log_x{0};
    std::array<double, iterations> log_e{0};
    std::array<double, iterations> log_u{0};
    std::array<double, iterations> log_v{0};
    std::array<double, iterations> log_t{0};
    std::array<double, iterations> log_fric{0};

    for(int i = 0; i < iterations; ++i){
        log_x[i] = x_current;
        log_v[i] = *v_current;
        log_t[i] = i * dt;
        u = controller.run();
        log_e[i] = controller.getError();
        // if(abs(v_current) < 0.001 && abs(u) < 0.3){ // Friction
        //     u = 0;
        // }
        log_u[i] = u;
        f_friction = 0.3 * *v_current * *v_current * m; // Friction Force
        if(v_current > 0){
            f_friction *= -1;
        }
        log_fric[i] = f_friction;
        a_current = (u+f_friction)/m;
        *v_current += a_current * dt;
        x_current += *v_current * dt;
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
        log["fric"][i] = log_fric[i];
    }
    writeToFile("../log.json", log);
}

int main(int argc, char const *argv[])
{
    desired_x();
    // desired_v();
    std::cout << "done." << std::endl;
    return 0;
}
