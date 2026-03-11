/**
 * Scrollwheel Advanced Kinematics & Gestures Example
 * * This example demonstrates how to process raw capacitive slider data into
 * high-level UI kinematics. It implements 5 key mathematical algorithms:
 * 1. Circular Wrap-Around Correction: Converts 0-359 deg to a continuous angle.
 * 2. 1-Euro Filter: An adaptive low-pass filter to eliminate jitter.
 * 3. Linear Regression (Velocity): Calculates a smooth derivative (velocity).
 * 4. Exponential Decay (Momentum): Simulates physical friction upon release.
 * 5. Hooke's Law (Rubber-Banding): Provides physical resistance when pushing
 * past list boundaries, and a smooth, critically-damped snap-back.
 */

#include <Arduino.h>
#include "Scrollwheel.h"

// ==============================================================================
// --- ⚙️ CONFIGURATION & TUNING PARAMETERS ---
// ==============================================================================

// Hardware Pinning
constexpr int kInterruptPin = 2;

// Virtual List Boundaries
// Define the start and end of your virtual UI (e.g., 0 to 1000 pixels/items)
constexpr float kMinScrollPos = -1000.0f;
constexpr float kMaxScrollPos = 1000.0f;

// Kinematic Tuning
// General sensitivity multiplier (1.0 = 1 degree of rotation equals 1 unit of scroll)
constexpr float kScrollSensitivity = 1.0f;

// Momentum (Friction)
// Determines how long the wheel keeps spinning after a fast release.
// Higher values (e.g., 0.8) = slips like ice. Lower values (e.g., 0.1) = high friction.
constexpr float kMomentumTau = 0.2f;

// Rubber-Banding (Boundary Physics)
// Resistance: How hard it is to push past the boundary while actively scrolling.
// (e.g., 0.05 means after 20 units, movement is halved. Higher = harder wall)
constexpr float kRubberBandResistance = 0.95f;

// Restitution: How fast it snaps back to the boundary when released.
// Range 0.01 (very slow/sluggish) to 0.5 (snaps back instantly).
// Kept below 1.0 to prevent overshooting the boundary.
constexpr float kRubberBandRestitution = 0.015f;

// ==============================================================================
// --- ALGORITHM CLASSES ---
// ==============================================================================

// --- 1-EURO FILTER (ADAPTIVE LOW-PASS) ---
class OneEuroFilter
{
public:
    OneEuroFilter(float min_cutoff = 1.0f, float beta = 0.005f, float d_cutoff = 1.0f)
        : min_cutoff_(min_cutoff), beta_(beta), d_cutoff_(d_cutoff),
          x_prev_(0), dx_prev_(0), t_prev_(0), first_time_(true) {}

    float filter(float x, float t)
    {
        if (first_time_)
        {
            x_prev_ = x;
            t_prev_ = t;
            first_time_ = false;
            return x;
        }

        float dt = t - t_prev_;
        if (dt <= 0.0f)
            return x_prev_;

        float dx = (x - x_prev_) / dt;
        float alpha_d = smoothingFactor(dt, d_cutoff_);
        float edx = alpha_d * dx + (1.0f - alpha_d) * dx_prev_;

        float cutoff = min_cutoff_ + beta_ * abs(edx);
        float alpha = smoothingFactor(dt, cutoff);
        float hat_x = alpha * x + (1.0f - alpha) * x_prev_;

        x_prev_ = hat_x;
        dx_prev_ = edx;
        t_prev_ = t;

        return hat_x;
    }

    void reset() { first_time_ = true; }

private:
    float min_cutoff_, beta_, d_cutoff_;
    float x_prev_, dx_prev_, t_prev_;
    bool first_time_;

    float smoothingFactor(float dt, float cutoff)
    {
        float r = 2.0f * PI * cutoff * dt;
        return r / (r + 1.0f);
    }
};

// --- VELOCITY ESTIMATOR (LINEAR REGRESSION) ---
class VelocityEstimator
{
public:
    VelocityEstimator() : count_(0) {}

    void update(float pos, float t)
    {
        if (count_ < kWindowSize)
        {
            history_pos_[count_] = pos;
            history_t_[count_] = t;
            count_++;
        }
        else
        {
            for (int i = 0; i < kWindowSize - 1; i++)
            {
                history_pos_[i] = history_pos_[i + 1];
                history_t_[i] = history_t_[i + 1];
            }
            history_pos_[kWindowSize - 1] = pos;
            history_t_[kWindowSize - 1] = t;
        }
    }

    float getVelocity()
    {
        if (count_ < 2)
            return 0.0f;

        float sum_t = 0, sum_p = 0, sum_t2 = 0, sum_tp = 0;
        float t0 = history_t_[0];

        for (int i = 0; i < count_; i++)
        {
            float t = history_t_[i] - t0;
            float p = history_pos_[i];
            sum_t += t;
            sum_p += p;
            sum_t2 += (t * t);
            sum_tp += (t * p);
        }

        float denominator = (count_ * sum_t2) - (sum_t * sum_t);
        if (denominator == 0.0f)
            return 0.0f;

        return ((count_ * sum_tp) - (sum_t * sum_p)) / denominator;
    }

    void clear() { count_ = 0; }

private:
    static const int kWindowSize = 5;
    float history_pos_[kWindowSize];
    float history_t_[kWindowSize];
    int count_;
};

// ==============================================================================
// --- SYSTEM GLOBALS ---
// ==============================================================================

Scrollwheel wheel;
OneEuroFilter filter(1.0f, 0.005f);
VelocityEstimator vel_estimator;

float g_continuous_angle = 0.0f;
int g_last_raw_angle = -1;

float g_virtual_scroll_pos = 0.0f;

enum class State
{
    kIdle,
    kScrolling,
    kMomentum
};
State g_state = State::kIdle;

float g_momentum_v0 = 0.0f;
float g_momentum_start_time = 0.0f;

// ==============================================================================
// --- MAIN PROGRAM ---
// ==============================================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    if (!wheel.begin(kInterruptPin))
    {
        delay(500);
    }

    wheel.setUpdateMode(UpdateMode::kHybrid, 10);
    Serial.println("Advanced Kinematics Initialized.");
}

void loop()
{
    // 1. TIMEKEEPING
    float current_time = millis() / 1000.0f;
    static float last_time = current_time;
    float dt = current_time - last_time;
    if (dt <= 0.0f)
        dt = 0.001f;
    last_time = current_time;

    // 2. READ HARDWARE
    bool has_new_data = wheel.update();
    int raw_angle = wheel.getSliderAngle();
    bool is_touched = wheel.isTouched();

    // 3. WRAP-AROUND CORRECTION
    if (is_touched && raw_angle >= 0)
    {
        if (g_last_raw_angle >= 0)
        {
            int diff = raw_angle - g_last_raw_angle;
            if (diff > 180)
                diff -= 360;
            else if (diff < -180)
                diff += 360;
            g_continuous_angle += diff;
        }
        g_last_raw_angle = raw_angle;
    }
    else
    {
        g_last_raw_angle = -1;
    }

    // 4. FILTERING & VELOCITY
    float filtered_angle = g_continuous_angle;
    float velocity = 0.0f;
    static float last_valid_velocity = 0.0f;

    if (is_touched)
    {
        filtered_angle = filter.filter(g_continuous_angle, current_time);
        if (has_new_data)
        {
            vel_estimator.update(filtered_angle, current_time);
        }
        velocity = vel_estimator.getVelocity();
        last_valid_velocity = velocity;
    }
    else
    {
        filter.reset();
        vel_estimator.clear();
        velocity = 0.0f;
    }

    // 5. STATE MACHINE & PHYSICS ENGINE
    switch (g_state)
    {
    case State::kIdle:
        if (is_touched)
            g_state = State::kScrolling;
        break;

    case State::kScrolling:
        if (is_touched)
        {
            static float last_filtered_angle = filtered_angle;
            float delta = filtered_angle - last_filtered_angle;
            last_filtered_angle = filtered_angle;

            float active_delta = delta * kScrollSensitivity;

            // --- RUBBER-BAND RESISTANCE DURING ACTIVE SCROLLING ---
            //
            // If the user pushes past the bounds, we scale down the delta
            // based on how far out of bounds they are.
            if (g_virtual_scroll_pos < kMinScrollPos)
            {
                float overtravel = kMinScrollPos - g_virtual_scroll_pos;
                active_delta *= (1.0f / (1.0f + overtravel * kRubberBandResistance));
            }
            else if (g_virtual_scroll_pos > kMaxScrollPos)
            {
                float overtravel = g_virtual_scroll_pos - kMaxScrollPos;
                active_delta *= (1.0f / (1.0f + overtravel * kRubberBandResistance));
            }

            g_virtual_scroll_pos += active_delta;
        }
        else
        {
            // Determine if we have enough exit velocity to trigger momentum
            if (abs(last_valid_velocity) > 40.0f)
            {
                g_state = State::kMomentum;
                g_momentum_v0 = last_valid_velocity * kScrollSensitivity;
                g_momentum_start_time = current_time;
            }
            else
            {
                g_state = State::kIdle;
            }
        }
        break;

    case State::kMomentum:
        if (is_touched)
        {
            g_state = State::kScrolling; // Catching the spinning wheel
        }
        else
        {
            float t = current_time - g_momentum_start_time;
            float current_vel = g_momentum_v0 * exp(-t / kMomentumTau);

            g_virtual_scroll_pos += (current_vel * dt);

            if (abs(current_vel) < 5.0f)
            {
                g_state = State::kIdle;
            }
        }
        break;
    }

    // 6. RUBBER-BAND SNAP-BACK
    // If not touched and out of bounds, smoothly pull it back to the limit.
    // Proportional control inherently accelerates at first, then decelerates
    // to zero as it reaches the target, preventing overshoot.
    if (!is_touched)
    {
        if (g_virtual_scroll_pos < kMinScrollPos)
        {

            // Abort momentum if we shoot out of bounds during a spin
            if (g_state == State::kMomentum)
                g_state = State::kIdle;

            g_virtual_scroll_pos += kRubberBandRestitution * (kMinScrollPos - g_virtual_scroll_pos);
        }
        else if (g_virtual_scroll_pos > kMaxScrollPos)
        {

            if (g_state == State::kMomentum)
                g_state = State::kIdle;

            g_virtual_scroll_pos += kRubberBandRestitution * (kMaxScrollPos - g_virtual_scroll_pos);
        }
    }

    // 7. DATA TELEMETRY OUTPUT
    bool is_pressed = wheel.isButtonPressed();
    static unsigned long last_print = 0;

    if (millis() - last_print > 30)
    {
        Serial.print("FilteredAngle:");
        Serial.print(filtered_angle);
        Serial.print(", Velocity:");
        Serial.print(last_valid_velocity);
        Serial.print(", VirtualPos:");
        Serial.print(g_virtual_scroll_pos);
        Serial.print(", State:");
        Serial.print(static_cast<int>(g_state));
        Serial.print(", Btn:");
        Serial.println(is_pressed ? 1 : 0);
        last_print = millis();
    }
}