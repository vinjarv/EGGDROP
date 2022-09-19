#pragma once

class PID {
public:
  PID(float Kp, float Ki, float Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd) {
    t_prev = micros() * 1.0e-6;
    pv_prev = 0;
  }

  // PID update function
  float run(const float sp, const float pv) {
    float u;
    float e = sp - pv;
//    Serial.print("PID SP:" );
//    Serial.print(sp);
//    Serial.print(" PV:");
//    Serial.println(pv);
    float t = micros() * 1.0e-6;
    float dt = t - t_prev;
    // Calculate pid output
    u = Kp_ * e + Ki_ * e_acc + Kd_ * (pv - pv_prev) / dt;
    // Output saturation and integrator anti-windup
    if (output_bounds and (u <= out_min_ or u >= out_max_)){
      u = constrain(u, out_min_, out_max_);
    } else {
      // Add integrator if not saturated
      e_acc += e * dt;
    }
    pv_prev = pv;
    t_prev = t;
    return u;
  }

  // Specify saturation values for output
  // Will also limit integrator to mitigate windup
  void set_output_bounds(const float lower, const float upper) {
    if (lower == 0 and upper == 0){
      output_bounds = false;
    }else{
      output_bounds = true;
      out_min_ = lower;
      out_max_ = upper;
    }
  }

private:
  const float Kp_, Ki_, Kd_;
  float t_prev, pv_prev;
  double e_acc;
  float out_min_, out_max_;
  bool output_bounds = false;
};
