#include "PositionPID.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>
#include <WiFi.h>
#include <cmath>
#include <esp_now.h>

const uint8_t PIN_DIR_1 = 18;
const uint8_t PIN_PWM_1 = 17;
const uint8_t PIN_DIR_2 = 21;
const uint8_t PIN_PWM_2 = 19;
const uint8_t PIN_DIR_3 = 23;
const uint8_t PIN_PWM_3 = 22;

const uint8_t W1_CH = 0;
const uint8_t W2_CH = 1;
const uint8_t W3_CH = 2;

const int8_t W1_SIGN = -1;
const int8_t W2_SIGN = -1;
const int8_t W3_SIGN = -1;

const double USER_SIGN = 1.0;

const double ENC_RESOLUTION        = 4096;
const double OD_RADIUS             = 30;
const double ROBOT_TO_ODO_RADIUS   = 210;
const double ROBOT_TO_WHEEL_RADIUS = 350;
const double GEAR_RATIO            = 1.0;

const double W1_ANGLE = 0.;
const double W2_ANGLE = 120.;
const double W3_ANGLE = 240.;

const double COUNTS_PER_MM = (ENC_RESOLUTION * GEAR_RATIO) / (PI * OD_RADIUS * 2);

ESP32Encoder enc1, enc2, enc3;

const double MAX_VX_CMD_MM_S     = 200.0;
const double MAX_VY_CMD_MM_S     = 200.0;
const double INTEGRAL_MAX        = 10.0;
const double MAX_OMEGA_CMD_RAD_S = 2.0; // 角速度上限[rad/s]

PositionPid x_pos_pid(0.7, 1., 0.0, -MAX_VX_CMD_MM_S, MAX_VX_CMD_MM_S, -INTEGRAL_MAX, INTEGRAL_MAX);
PositionPid y_pos_pid(0.7, 1., 0.0, -MAX_VY_CMD_MM_S, MAX_VY_CMD_MM_S, -INTEGRAL_MAX, INTEGRAL_MAX);
PositionPid theta_pos_pid(0.7, 1., 0.0, -MAX_OMEGA_CMD_RAD_S, MAX_OMEGA_CMD_RAD_S, -INTEGRAL_MAX, INTEGRAL_MAX);
struct Position {
        double x;
        double y;
        double theta;
};

Position targetPos{0, 0, 0};

// 受信共有バッファとロック（追加）
portMUX_TYPE     targetMux     = portMUX_INITIALIZER_UNLOCKED;
volatile int16_t rx_x_mm       = 0;
volatile int16_t rx_y_mm       = 0;
volatile int16_t rx_theta_mrad = 0;

double WrapPi(double rad) {
    while (rad > PI)
        rad -= 2.0 * PI;
    while (rad < -PI)
        rad += 2.0 * PI;
    return rad;
}

class Motor {

    public:
        Motor(uint8_t pinDir, uint8_t pinPwm, uint8_t pwmCh, int8_t sign)
            : pinDir_(pinDir), pinPwm_(pinPwm), pwmCh_(pwmCh), direction_(sign) {}

        void begin() {
            pinMode(pinDir_, OUTPUT);
            ledcSetup(pwmCh_, 12800, 8);
            ledcAttachPin(pinPwm_, pwmCh_);
        }

        void Write(int16_t pwm_signed) {
            int duty = direction_ * pwm_signed;
            if (abs(duty) < PWM_DEADBAND) {
                ledcWrite(pwmCh_, 0);
                return;
            }
            duty = constrain(duty, -255, 255);
            digitalWrite(pinDir_, (duty > 0) ? HIGH : LOW);
            ledcWrite(pwmCh_, abs(duty));
        }

    private:
        uint8_t              pinDir_;
        uint8_t              pinPwm_;
        uint8_t              pwmCh_;
        int8_t               direction_;
        static const uint8_t PWM_DEADBAND = 8;
};

class Odometry {
    public:
        Odometry(ESP32Encoder& e1, ESP32Encoder& e2, ESP32Encoder& e3) : enc1_(e1), enc2_(e2), enc3_(e3) {}

        void begin() {
            ESP32Encoder::useInternalWeakPullResistors = puType::up;

            enc1_.attachHalfQuad(PIN_ROTARY_A_1, PIN_ROTARY_B_1);
            enc2_.attachHalfQuad(PIN_ROTARY_A_2, PIN_ROTARY_B_2);
            enc3_.attachHalfQuad(PIN_ROTARY_A_3, PIN_ROTARY_B_3);

            enc1_.clearCount();
            enc2_.clearCount();
            enc3_.clearCount();

            prev_count1_ = enc1_.getCount();
            prev_count2_ = enc2_.getCount();
            prev_count3_ = enc3_.getCount();

            pos_ = {0.0, 0.0, 0.0};

            buildInverse_();
        }

        void update() {
            if (!inv_ok_) buildInverse_();
            if (!inv_ok_) return;

            const long c1 = enc1_.getCount();
            const long c2 = enc2_.getCount();
            const long c3 = enc3_.getCount();

            const long dc1 = (c1 - prev_count1_) * ENCODER_SIGN_1;
            const long dc2 = (c2 - prev_count2_) * ENCODER_SIGN_2;
            const long dc3 = (c3 - prev_count3_) * ENCODER_SIGN_3;

            prev_count1_ = c1;
            prev_count2_ = c2;
            prev_count3_ = c3;

            const Position dpos = CountToBody(dc1, dc2, dc3);

            const double th_mid = pos_.theta + 0.5 * dpos.theta;
            const double ct_mid = cos(th_mid);
            const double st_mid = sin(th_mid);

            pos_.x += ct_mid * dpos.x - st_mid * dpos.y;
            pos_.y += st_mid * dpos.x + ct_mid * dpos.y;
            pos_.theta = WrapPi(pos_.theta + dpos.theta);
        }

        Position get_position() const { return pos_; }

    private:
        static constexpr uint8_t PIN_ROTARY_A_1 = 25;
        static constexpr uint8_t PIN_ROTARY_B_1 = 26;
        static constexpr uint8_t PIN_ROTARY_A_2 = 32;
        static constexpr uint8_t PIN_ROTARY_B_2 = 33;
        static constexpr uint8_t PIN_ROTARY_A_3 = 27;
        static constexpr uint8_t PIN_ROTARY_B_3 = 14;

        static constexpr int8_t ENCODER_SIGN_1 = 1;
        static constexpr int8_t ENCODER_SIGN_2 = 1;
        static constexpr int8_t ENCODER_SIGN_3 = 1;

        ESP32Encoder& enc1_;
        ESP32Encoder& enc2_;
        ESP32Encoder& enc3_;

        long prev_count1_{0};
        long prev_count2_{0};
        long prev_count3_{0};

        Position pos_{0., 0., 0.};

        bool   inv_ok_{false};
        double invA_[3][3]{};

        static bool Invert3x3(const double A[3][3], double invA[3][3]) {

            // 行列式
            const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                               A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                               A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

            // 1e-12は不動点数誤差を考慮した無視できる値
            if (fabs(det) < 1e-12) return false;
            const double invDet = 1.0 / det;

            invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
            invA[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
            invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

            invA[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
            invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
            invA[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

            invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
            invA[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
            invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

            return true;
        }

        void buildInverse_() {
            const double a1 = 135.0 * PI / 180.0;
            const double a2 = 225.0 * PI / 180.0;
            const double a3 = 0.0 * PI / 180.0;

            const double c1 = cos(a1), sn1 = sin(a1);
            const double c2 = cos(a2), sn2 = sin(a2);
            const double c3 = cos(a3), sn3 = sin(a3);

            // 回転寄与の符号を working code に合わせる
            const double k = -ROBOT_TO_ODO_RADIUS;

            const double A[3][3] = {
                {c1, sn1, k},
                {c2, sn2, k},
                {c3, sn3, k},
            };

            inv_ok_ = Invert3x3(A, invA_);
        }

        Position WheelDeltaToBodyDelta(double s1_mm, double s2_mm, double s3_mm) const {
            if (!inv_ok_) return {0.0, 0.0, 0.0};

            const double dx     = invA_[0][0] * s1_mm + invA_[0][1] * s2_mm + invA_[0][2] * s3_mm;
            const double dy     = invA_[1][0] * s1_mm + invA_[1][1] * s2_mm + invA_[1][2] * s3_mm;
            const double dtheta = invA_[2][0] * s1_mm + invA_[2][1] * s2_mm + invA_[2][2] * s3_mm;

            return {dx, dy, dtheta};
        }

        Position CountToBody(long dc1, long dc2, long dc3) const {
            const double s1 = (double)dc1 / COUNTS_PER_MM;
            const double s2 = (double)dc2 / COUNTS_PER_MM;
            const double s3 = (double)dc3 / COUNTS_PER_MM;
            return WheelDeltaToBodyDelta(s1, s2, s3);
        }
};

class OmniWheel {
    public:
        OmniWheel(Motor& motor, double angle, double pwm_gain = 1.0)
            : motor_(motor), angle_(angle / 180 * PI), gain_(pwm_gain) {}

        void run(double vx_mm_s, double vy_mm_s, double omega_rad_s) {
            const double u_mm_s = cos(angle_) * vx_mm_s + sin(angle_) * vy_mm_s + ROBOT_TO_WHEEL_RADIUS * omega_rad_s;

            const int pwm = (int)lround(u_mm_s * gain_);
            motor_.Write((int16_t)constrain(pwm, -PWM_LIMIT, PWM_LIMIT));
        }

    private:
        Motor&                   motor_;
        double                   angle_{0.0};
        double                   gain_{1.0};
        static constexpr int16_t PWM_LIMIT = 200;
};

class OmniWheels {
    public:
        OmniWheels(OmniWheel& w1, OmniWheel& w2, OmniWheel& w3) : w1_(w1), w2_(w2), w3_(w3) {}

        void moveDelta(double dx_mm, double dy_mm, double dtheta_rad, double dt_s) {
            if (dt_s <= 0.0) return;
            double dx_mm_s      = dx_mm / dt_s;
            double dy_mm_s      = dy_mm / dt_s;
            double dtheta_rad_s = dtheta_rad / dt_s;

            move(dx_mm_s, dy_mm_s, dtheta_rad_s);
        }

        void move(double vx_mm_s, double vy_mm_s, double omega_rad_s) {
            w1_.run(vx_mm_s, vy_mm_s, omega_rad_s);
            w2_.run(vx_mm_s, vy_mm_s, omega_rad_s);
            w3_.run(vx_mm_s, vy_mm_s, omega_rad_s);
        }

    private:
        OmniWheel& w1_;
        OmniWheel& w2_;
        OmniWheel& w3_;
};

class EspNowSend {
    public:
        void begin() {
            WiFi.mode(WIFI_STA);
            WiFi.disconnect();

            if (esp_now_init() == ESP_OK) {
                Serial.println("ESPNow Init Success");
            } else {
                Serial.println("ESPNow Init Failed");
                ESP.restart();
            }

            memset(&slave, 0, sizeof(slave));

            uint8_t peerAddress[] = {0x08, 0xD1, 0xF9, 0x37, 0x41, 0xF0};
            memcpy(slave.peer_addr, peerAddress, 6);

            // チャンネル指定(0はチャンネル自動)
            slave.channel = 0;
            // 暗号化しない
            slave.encrypt = false;

            esp_err_t addStatus = esp_now_add_peer(&slave);
            if (addStatus == ESP_OK) {
                // Pair success
                Serial.println("Pair success");
            }

            esp_now_register_send_cb(OnDataSent);
            esp_now_register_recv_cb(OnDataRecv);
        }
        // esp_err_t send(int16_t send_x_mm, int16_t send_y_mm, int16_t send_theta_mrad, int16_t send_w1_t, int16_t send_w2_t,
        //                int16_t send_w3_t, int16_t send_pwm1, int16_t send_pwm2, int16_t send_pwm3) {
        esp_err_t send(int16_t send_x_mm, int16_t send_y_mm, int16_t send_theta_mrad) {
            uint8_t data[] = {
                (uint8_t)((send_x_mm >> 8) & 0xFF),       (uint8_t)(send_x_mm & 0xFF),
                (uint8_t)((send_y_mm >> 8) & 0xFF),       (uint8_t)(send_y_mm & 0xFF),
                (uint8_t)((send_theta_mrad >> 8) & 0xFF), (uint8_t)(send_theta_mrad & 0xFF),
                // (uint8_t)((send_w1_t >> 8) & 0xFF),       (uint8_t)(send_w1_t & 0xFF),
                // (uint8_t)((send_w2_t >> 8) & 0xFF),       (uint8_t)(send_w2_t & 0xFF),
                // (uint8_t)((send_w3_t >> 8) & 0xFF),       (uint8_t)(send_w3_t & 0xFF),
                // (uint8_t)((send_pwm1 >> 8) & 0xFF),       (uint8_t)(send_pwm1 & 0xFF),
                // (uint8_t)((send_pwm2 >> 8) & 0xFF),       (uint8_t)(send_pwm2 & 0xFF),
                // (uint8_t)((send_pwm3 >> 8) & 0xFF),       (uint8_t)(send_pwm3 & 0xFF),
            };
            esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
            return result;
        }

    private:
        esp_now_peer_info_t slave;

        static void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {}

        static void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
            if (data_len < 6) return;

            // uint8_t を uint16_t に昇格してからシフトして符号拡張を防ぐ
            int16_t tx = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
            int16_t ty = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);
            int16_t tt = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);

            portENTER_CRITICAL(&targetMux);
            rx_x_mm       = tx;
            rx_y_mm       = ty;
            rx_theta_mrad = tt;
            portEXIT_CRITICAL(&targetMux);
        }
};

class Robot {
    public:
        Robot(OmniWheels& wheels, Odometry& odo, EspNowSend& esp_now) : omni_wheels(wheels), odometry(odo), esp_now(esp_now) {}

        void set_target(Position position) { target = position; }

        void update(double dt) {
            int16_t cx, cy, cth;
            portENTER_CRITICAL(&targetMux);
            cx  = rx_x_mm;
            cy  = rx_y_mm;
            cth = rx_theta_mrad;
            portEXIT_CRITICAL(&targetMux);

            target.x     = (double)cx;
            target.y     = (double)cy;
            target.theta = WrapPi((double)cth / 1000.0);

            const Position nowPos = odometry.get_position();

            const double ex       = target.x - nowPos.x;
            const double ey       = target.y - nowPos.y;
            const double dist_err = hypot(ex, ey);
            const double th_err   = WrapPi(target.theta - nowPos.theta);

            double vx_world = 0;
            double vy_world = 0;
            double omega    = 0;

            const bool in_deadzone = (dist_err < POS_DEADZONE_MM) && (fabs(th_err) < THETA_DEADZONE_RAD);

            if (!in_deadzone) {
                vx_world = x_pos_pid.update(target.x, nowPos.x, dt);
                vy_world = y_pos_pid.update(target.y, nowPos.y, dt);
                omega    = theta_pos_pid.update(target.theta, nowPos.theta, dt);
            } else {
                x_pos_pid.reset();
                y_pos_pid.reset();
                theta_pos_pid.reset();
            }

            const double ct      = cos(nowPos.theta);
            const double st      = sin(nowPos.theta);
            const double vx_body = ct * vx_world + st * vy_world;
            const double vy_body = -st * vx_world + ct * vy_world;

            static unsigned long last_send_ms = 0;
            unsigned long        ms           = millis();
            if ((long)(ms - last_send_ms) >= 100) {
                last_send_ms = ms;
                esp_err_t res =
                    esp_now.send((int16_t)lround(nowPos.x), (int16_t)lround(nowPos.y), (int16_t)lround(nowPos.theta * 1000.0));
                if (res != ESP_OK) {
                    // 必要ならログ
                    Serial.printf("esp_now_send err=%d\n", res);
                }
            }

            omni_wheels.move(vx_body, vy_body, omega);
        }

    private:
        OmniWheels& omni_wheels;
        Odometry&   odometry;
        EspNowSend& esp_now;
        Position    target{0., 0., 0.};

        double x_control     = 0;
        double y_control     = 0;
        double theta_control = 0;

        const double POS_DEADZONE_MM    = 5.0;               // 変更: 10->5 mm
        const double THETA_DEADZONE_RAD = 3.0 * PI / 180.0;  // 変更: 10deg->3deg
};

Motor m1(PIN_DIR_1, PIN_PWM_1, W1_CH, W1_SIGN);
Motor m2(PIN_DIR_2, PIN_PWM_2, W2_CH, W2_SIGN);
Motor m3(PIN_DIR_3, PIN_PWM_3, W3_CH, W3_SIGN);

Odometry odometry(enc1, enc2, enc3);

OmniWheel w1(m1, W1_ANGLE);
OmniWheel w2(m2, W2_ANGLE);
OmniWheel w3(m3, W3_ANGLE);

OmniWheels omuni(w1, w2, w3);

EspNowSend esp_now;

Robot robot(omuni, odometry, esp_now);

constexpr long US_CONTROL_CYCLE = 5000;

unsigned long last = micros();

void setup() {
    Serial.begin(115200);
    m1.begin();
    m2.begin();
    m3.begin();
    odometry.begin();
    esp_now.begin();

    last = micros();
}

void loop() {
    unsigned long now = micros();

    if (now - last < US_CONTROL_CYCLE) return;
    double dt = (now - last) * 1.e-6;
    last      = now;

    odometry.update();
    robot.update(dt);
}
