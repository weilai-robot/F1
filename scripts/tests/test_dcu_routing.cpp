// Run the production DCU and motor encoders without an EtherCAT interface.
// Input rows: motor,ecat_id,channel,can_id,type,neutral,target,kp,kd.
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "internal/dcu.h"
#include "internal/power_flow.h"

namespace xyber_utils {
void FloatToBytes(float value, uint8_t* bytes) { std::memcpy(bytes, &value, sizeof(value)); }
float BytesToFloat(const uint8_t* bytes) {
  float value;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}
}  // namespace xyber_utils

namespace {
struct Case {
  std::string name, type;
  unsigned slave, channel, id;
  float neutral, target, kp, kd;
};
void Require(bool condition, const std::string& message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
using Packet = std::array<uint8_t, sizeof(xyber::DcuSendPacket)>;
Packet Snapshot(xyber::Dcu& dcu) {
  Packet packet{};
  dcu.PullSendData(packet.data(), packet.size());
  return packet;
}
}  // namespace

int main(int argc, char** argv) {
  const xyber::MitParam r86_param = PF_R86_MIT_MODE_DEFAULT_PARAM;
  const xyber::MitParam r52_param = PF_R52_MIT_MODE_DEFAULT_PARAM;
  Require(argc == 2, "expected a CSV case file");
  std::ifstream input(argv[1]);
  Require(input.good(), "cannot read cases");
  std::vector<Case> cases;
  std::string line;
  while (std::getline(input, line)) {
    std::stringstream stream(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    Require(fields.size() == 9, "invalid CSV row");
    cases.push_back({fields[0], fields[4], unsigned(std::stoul(fields[1])),
      unsigned(std::stoul(fields[2])), unsigned(std::stoul(fields[3])),
      std::stof(fields[5]), std::stof(fields[6]), std::stof(fields[7]), std::stof(fields[8])});
  }
  Require(cases.size() == 17, "expected all 17 upper-body actuators");
  std::array<std::unique_ptr<xyber::Dcu>, 256> buses;
  for (const auto& c : cases) {
    if (!buses[c.slave]) buses[c.slave] = std::make_unique<xyber::Dcu>("test", c.slave);
    auto channel = static_cast<xyber::CtrlChannel>(c.channel - 1);
    xyber::Actuator* motor;
    if (c.type == "POWER_FLOW_L28") {
      motor = new xyber_power_flow::PowerFlowL(c.name, c.id, channel);
    } else {
      const bool r86 = c.type == "POWER_FLOW_R86";
      motor = new xyber_power_flow::PowerFlowR(r86 ? xyber::ActuatorType::POWER_FLOW_R86 : xyber::ActuatorType::POWER_FLOW_R52,
        c.name, c.id, channel, r86 ? r86_param : r52_param);
    }
    buses[c.slave]->RegisterActuator(motor);
  }
  for (const auto& selected : cases) {
    for (const auto& c : cases) buses[c.slave]->SetMitCmd(c.name, c.neutral, 0, 0, c.kp, c.kd);
    std::array<Packet, 256> before{};
    for (unsigned i = 1; i < buses.size(); ++i) if (buses[i]) before[i] = Snapshot(*buses[i]);
    buses[selected.slave]->SetMitCmd(selected.name, selected.target, 0, 0, selected.kp, selected.kd);
    const size_t offset = (selected.channel - 1) * 65 + 1 + (selected.id - 1) * 8;
    bool changed = false;
    Packet selected_packet{};
    for (unsigned i = 1; i < buses.size(); ++i) {
      if (!buses[i]) continue;
      auto after = Snapshot(*buses[i]);
      if (i == selected.slave) selected_packet = after;
      for (size_t byte = 0; byte < after.size(); ++byte) {
        const bool intended = i == selected.slave && byte >= offset && byte < offset + 8;
        if (intended) changed |= after[byte] != before[i][byte];
        else Require(after[byte] == before[i][byte], selected.name + ": changed another motor/channel");
      }
    }
    Require(changed, selected.name + ": no motor frame changed");
    Require(selected_packet[(selected.channel - 1) * 65] == 255, selected.name + ": CAN broadcast mask not set");
    const uint8_t* frame = selected_packet.data() + offset;
    if (selected.type == "POWER_FLOW_L28") {
      Require(std::abs(xyber_utils::BytesToFloat(frame) - selected.target) < 1e-6,
              selected.name + ": L28 target wrong");
    } else {
      const xyber::MitParam p = selected.type == "POWER_FLOW_R86" ? r86_param : r52_param;
      const auto decode = [](unsigned value, float lo, float hi, unsigned count) {
        return lo + value * (hi - lo) / count;
      };
      Require(std::abs(decode((frame[0] << 8) | frame[1], p.pos_min, p.pos_max, 65535) - selected.target)
                <= (p.pos_max - p.pos_min) / 65535 + 1e-6, selected.name + ": MIT position wrong");
      Require(std::abs(decode(((frame[3] & 15) << 8) | frame[4], p.kp_min, p.kp_max, 4095) - selected.kp)
                <= (p.kp_max - p.kp_min) / 4095 + 1e-6, selected.name + ": MIT kp wrong");
      Require(std::abs(decode((frame[5] << 4) | (frame[6] >> 4), p.kd_min, p.kd_max, 4095) - selected.kd)
                <= (p.kd_max - p.kd_min) / 4095 + 1e-6, selected.name + ": MIT kd wrong");
    }
    xyber::DcuRecvPacket received{};
    uint8_t* feedback = received.canfd[selected.channel - 1] + (selected.id - 1) * 8;
    if (selected.type == "POWER_FLOW_L28") {
      std::memcpy(feedback, &selected.target, sizeof(selected.target));
      const float current = 0.75f;
      std::memcpy(feedback + sizeof(float), &current, sizeof(current));
    } else {
      feedback[0] = frame[0];
      feedback[1] = frame[1];
    }
    buses[selected.slave]->PushRecvData(reinterpret_cast<const uint8_t*>(&received), sizeof(received));
    Require(std::abs(buses[selected.slave]->GetPosition(selected.name) - selected.target) < 0.0004f,
            selected.name + ": feedback read from wrong CAN slot");
    if (selected.type == "POWER_FLOW_L28") {
      Require(buses[selected.slave]->GetEffort(selected.name) == 0.75f,
              selected.name + ": L28 feedback current wrong");
    }
    std::cout << "PASS " << selected.name << " ECAT " << selected.slave
              << " channel " << selected.channel << " CAN " << selected.id << '\n';
  }
}
