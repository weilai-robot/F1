// Exercise the production PowerFlowR encoder without EtherCAT or ROS2.
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

#include "internal/power_flow.h"

// power_flow.cpp also references this utility from its float-command methods.
// Supply the byte-copy utility so this test does not link Linux RT scheduling.
namespace xyber_utils {
void FloatToBytes(const float value, uint8_t* bytes) {
  std::memcpy(bytes, &value, sizeof(value));
}
}  // namespace xyber_utils

namespace {
using Frame = std::array<uint8_t, ACTUATOR_FRAME_SIZE>;
using Command = std::array<float, 5>;  // position, velocity, effort, kp, kd

Frame Encode(const xyber::MitParam& param, const Command& cmd) {
  Frame send{}, recv{};
  xyber_power_flow::PowerFlowR motor(xyber::ActuatorType::POWER_FLOW_R52, "test", 1,
                                   xyber::CtrlChannel::CTRL_CH1, param);
  static_cast<xyber::Actuator&>(motor).SetDataFiled(send.data(), recv.data());
  motor.SetMitCmd(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
  return send;
}

Command Decode(const xyber::MitParam& p, const Frame& b) {
  const auto from_wire = [](unsigned code, float lo, float hi, unsigned max) {
    return lo + code * (hi - lo) / max;
  };
  return {from_wire((b[0] << 8) | b[1], p.pos_min, p.pos_max, 65535),
          from_wire((b[2] << 4) | (b[3] >> 4), p.vel_min, p.vel_max, 4095),
          from_wire(((b[6] & 15) << 8) | b[7], p.toq_min, p.toq_max, 4095),
          from_wire(((b[3] & 15) << 8) | b[4], p.kp_min, p.kp_max, 4095),
          from_wire((b[5] << 4) | (b[6] >> 4), p.kd_min, p.kd_max, 4095)};
}

void Require(bool ok, const char* message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void CheckMotor(const xyber::MitParam& p) {
  const Command lo{p.pos_min, p.vel_min, p.toq_min, p.kp_min, p.kd_min};
  const Command hi{p.pos_max, p.vel_max, p.toq_max, p.kp_max, p.kd_max};
  const Command base{0.25f, p.vel_min, 0.0f, 120.0f, 3.3f};
  const float extreme = std::numeric_limits<float>::max();

  // Saturation must equal a command at the physical endpoint, byte for byte.
  // Comparing the entire frame also detects corruption of adjacent fields.
  for (size_t i = 0; i < base.size(); ++i) {
    Command outside = base, endpoint = base;
    outside[i] = extreme;
    endpoint[i] = hi[i];
    Require(Encode(p, outside) == Encode(p, endpoint), "positive overflow must saturate");
    outside[i] = -extreme;
    endpoint[i] = lo[i];
    Require(Encode(p, outside) == Encode(p, endpoint), "negative overflow must saturate");
  }

  // Concrete deployment regressions: sim gains and R52/R86 effort overflow.
  for (const Command cmd : {Command{0, 0, 0, 700, 5}, Command{0, 0, 0, 100, 10},
                            Command{0, p.vel_min, 0, 500.2f, 5},
                            Command{0, 0, 0, 150, 8.01f},
                            Command{0, 0, 1.6f * p.toq_max, 0, 0},
                            Command{0, 0, 1.6f * p.toq_min, 0, 0}}) {
    Command endpoint = cmd;
    for (size_t i = 0; i < cmd.size(); ++i) {
      endpoint[i] = std::fmax(lo[i], std::fmin(hi[i], cmd[i]));
    }
    Require(Encode(p, cmd) == Encode(p, endpoint), "deployment overflow regression");
  }

  // In-range gain and effort values must survive the wire representation to
  // within one quantization step, including endpoints and AMP/stand gains.
  for (unsigned n = 0; n <= 1000; ++n) {
    Command cmd{};
    for (size_t i = 0; i < cmd.size(); ++i) {
      cmd[i] = lo[i] + (hi[i] - lo[i]) * (n / 1000.0f);
    }
    const Command decoded = Decode(p, Encode(p, cmd));
    for (size_t i = 0; i < cmd.size(); ++i) {
      const float step = (hi[i] - lo[i]) / (i == 0 ? 65535.0f : 4095.0f);
      Require(std::abs(decoded[i] - cmd[i]) <= step * 1.01f + 1e-5f,
              "in-range command exceeds one quantization step");
    }
  }
  for (const Command cmd : {base, Command{0, 0, 0, 150, 5}, Command{0, 0, 0, 500, 5}}) {
    const Command decoded = Decode(p, Encode(p, cmd));
    Require(std::abs(decoded[3] - cmd[3]) <= 500.0f / 4095 + 1e-5f, "normal kp");
    Require(std::abs(decoded[4] - cmd[4]) <= 8.0f / 4095 + 1e-5f, "normal kd");
  }
}
}  // namespace

int main() {
  CheckMotor(PF_R52_MIT_MODE_DEFAULT_PARAM);
  CheckMotor(PF_R86_MIT_MODE_DEFAULT_PARAM);
  std::cout << "PASS: R52/R86 MIT saturation, field isolation and in-range quantization\n";
}
