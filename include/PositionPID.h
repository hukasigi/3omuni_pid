#pragma once
#include <Arduino.h>

class PositionPid {
    private:
        double kp, ki, kd;
        double out_min, out_max;

        double integral;
        double integral_max, integral_min;
        double control;

        double prev_meas;
        double prev_error;
        bool   first;

        double error;

    public:
        PositionPid(double kp, double ki, double kd, double out_min, double out_max, double integral_min, double integral_max)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), integral(0.0), integral_min(integral_min),
              integral_max(integral_max), control(0.0), prev_meas(0.0), prev_error(0.0), first(true) {}

        // dt[s]
        double update(double target_pos, double now_pos, double dt) {
            if (dt <= 0.0) return control;

            error = target_pos - now_pos;

            integral += error * dt;
            integral = constrain(integral, integral_min, integral_max);

            double derivative = 0.0;
            if (!first) {
                // 誤差の微分を取る（符号は自然）
                derivative = (error - prev_error) / dt;
            } else {
                first = false;
            }
            prev_meas  = now_pos;
            prev_error = error;

            control = kp * error + ki * integral + kd * derivative;
            control = constrain(control, out_min, out_max);
            return control;
        }

        double getError() { return error; }

        void reset(double now_pos = 0.0) {
            integral   = 0.0;
            control    = 0.0;
            prev_meas  = now_pos;
            prev_error = 0.0;
            first      = true;
        }
};