#include "api.h"
#include <algorithm>

class PID
{
    private:
        double kP;
        double kI; 
        double kD;
        uint64_t time_now;
        uint64_t time_prev;
        double error_now;
        double error_prev;
        double error_int;
        const double MAX_I = 500.0;

    public:
        PID(double P, double I, double D){
            kP = P;
            kI = I;
            kD = D;
            time_now = -1;
            time_prev = -1;
            error_now = 0.0;
            error_prev = 0.0;
            error_int = 0.0; 
        }

        double step(double error){ //calculates the PID output based on the error and the PID constants
            if(time_prev == -1){
                time_prev = pros::micros();
                return 0.0;
            }
            time_now = pros::micros();
            double dt = (time_now-time_prev) / 1000.0;

            if (std::isnan(error)) {
                error_now = 0.0;
                error_prev = 0.0; 
            }else{
                error_prev = error_now;
                error_now = error;
            }

            if (std::isnan(error_int)) error_int = 0;
            error_int += error_now * dt;

            // enforce max threshold for error integral
            error_int = std::clamp(error_int, -MAX_I, MAX_I);

            // prevent iterm windup, set iterm to zero when error changes sign
            if (__builtin_signbitf(error_now) != __builtin_signbitf(error_prev)) error_int = 0.0;

            // calculate the D term
            double dterm = (kD * error_now - kD * error_prev) / dt;

            // calculate the PID output
            return 10.0 * (kP * error_now + kI * error_int + dterm);
        }
};