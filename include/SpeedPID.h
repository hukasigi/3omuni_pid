#pragma once
#include <Arduino.h>

class SpeedPID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double prev_error, prev_delta_error;
        double output;
        double error;
        double filt_deriv = 0.0; // メンバにする

    public:
        SpeedPID(double kp, double ki, double kd, double out_min, double out_max)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), prev_error(0.0), prev_delta_error(0.0), output(0.0) {}

        double update(double target, double measurement, double dt) {
            if (dt <= 0.0) return output;

            error = target - measurement;

            const double delta_error = error - prev_error;

            // 簡易微分ローパス（離散一次遅れ）
            constexpr double deriv_alpha = 0.1; // 0〜1 小さいほど強く平滑
            const double     raw_deriv   = (delta_error - prev_delta_error) / dt;
            filt_deriv += deriv_alpha * (raw_deriv - filt_deriv);

            double du = kp * delta_error + ki * error * dt + kd * filt_deriv;

            // アンチワインドアップ：飽和時に積分増分を抑制
            double tentative = output + du;
            if ((tentative > out_max && du > 0) || (tentative < out_min && du < 0)) {
                du -= ki * error * dt; // 積分分を打ち消す
            }

            output += du;
            output = constrain(output, out_min, out_max);

            prev_error       = error;
            prev_delta_error = delta_error;
            return output;
        }

        double getError() { return error; }

        void reset() {
            prev_error       = 0.;
            prev_delta_error = 0.;
            output           = 0.;
            filt_deriv       = 0.; // クリア
        }
};
