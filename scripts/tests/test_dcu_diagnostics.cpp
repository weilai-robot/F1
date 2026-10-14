#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include "internal/dcu.h"
#include "internal/power_flow.h"

namespace xyber_utils {
void FloatToBytes(float f,uint8_t* p) { std::memcpy(p,&f,4); }
float BytesToFloat(const uint8_t* p) { float f;std::memcpy(&f,p,4);return f; }
}
void Check(bool ok,const char* message) { if(!ok) { std::cerr<<message<<'\n';std::exit(1); } }
void Feedback(uint8_t* p,unsigned q,unsigned dq,unsigned tau,uint8_t status) {
  p[0]=q>>8;p[1]=q;p[2]=dq>>4;p[3]=(dq&15)<<4|(tau>>8);p[4]=tau;p[5]=status;
}
int main() {
  using namespace xyber;
  const MitParam params=PF_R86_MIT_MODE_DEFAULT_PARAM;
  Dcu dcu("body",1);
  auto* left=new xyber_power_flow::PowerFlowR(ActuatorType::POWER_FLOW_R86,"left",1,CtrlChannel::CTRL_CH1,params);
  auto* right=new xyber_power_flow::PowerFlowR(ActuatorType::POWER_FLOW_R86,"right",1,CtrlChannel::CTRL_CH2,params);
  auto* wrist=new xyber_power_flow::PowerFlowL("wrist",6,CtrlChannel::CTRL_CH1);
  Check(dcu.RegisterActuator(left)&&dcu.RegisterActuator(right)&&dcu.RegisterActuator(wrist),"valid routes");
  auto duplicate=std::make_unique<xyber_power_flow::PowerFlowR>(ActuatorType::POWER_FLOW_R86,"duplicate",1,CtrlChannel::CTRL_CH1,params);
  auto invalid=std::make_unique<xyber_power_flow::PowerFlowR>(ActuatorType::POWER_FLOW_R86,"invalid",9,CtrlChannel::CTRL_CH1,params);
  auto bad_channel=std::make_unique<xyber_power_flow::PowerFlowR>(ActuatorType::POWER_FLOW_R86,"bad_channel",1,static_cast<CtrlChannel>(3),params);
  Check(!dcu.RegisterActuator(duplicate.get())&&!dcu.RegisterActuator(invalid.get())&&!dcu.RegisterActuator(bad_channel.get()),"unsafe/duplicate routes rejected");
  ActuatorDiagnostics s;
  Check(dcu.GetActuatorDiagnostics("left",s)&&!s.rx_seen&&!s.tx_pdo_seq,"unseen snapshot");
  dcu.SetMitCmd("left",.3f,0,0,150,5);
  dcu.SetMitCmd("wrist",.25f,0,0,15,1);
  std::array<uint8_t,sizeof(DcuSendPacket)> output{};
  dcu.PullSendData(output.data(),output.size());
  Check(dcu.GetActuatorDiagnostics("left",s)&&s.tx_format==1&&s.tx_command_seq==1,"MIT tagged");
  Check(std::abs(s.cmd_position-.3f)<.0002f&&std::abs(s.cmd_kp-150)<.123f&&std::abs(s.cmd_kd-5)<.002f,"decode actual pulled MIT frame");
  dcu.SetMitCmd("left",.6f,0,0,120,4);
  dcu.GetActuatorDiagnostics("left",s);
  Check(std::abs(s.cmd_position-.3f)<.0002f&&s.tx_command_seq==1,"pending cache not misreported as sent");
  dcu.PullSendData(output.data(),output.size());
  dcu.GetActuatorDiagnostics("left",s);
  Check(std::abs(s.cmd_position-.6f)<.0002f&&s.tx_command_seq==2,"next pulled frame captured");
  dcu.GetActuatorDiagnostics("wrist",s);
  Check(s.tx_format==2&&s.cmd_position==.25f&&s.cmd_current_limit==2.5f&&std::isnan(s.cmd_kp),"L28 is position/current limit, not MIT gains");
  DcuRecvPacket packet{};
  Feedback(packet.canfd[0],36000,2400,1800,3);
  Feedback(packet.canfd[1],32000,2100,2200,3);
  const float wrist_q=.2f,wrist_cur=.7f;
  std::memcpy(packet.canfd[0]+40,&wrist_q,4);std::memcpy(packet.canfd[0]+44,&wrist_cur,4);
  dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));
  dcu.GetActuatorDiagnostics("left",s);
  const auto content=s.rx_content_seq,measurement=s.rx_measurement_seq;
  const auto stamp=s.rx_measurement_ns;
  ActuatorStateSnapshot first_state;
  dcu.GetActuatorState("left",first_state);
  Check(first_state.pdo_received_ns==s.rx_pdo_ns&&first_state.pdo_received_ns>0,"coherent state includes valid PDO timestamp");
  Check(s.rx_seen&&s.rx_pdo_seq==1&&s.rx[5]==3&&dcu.GetPowerState("left")==STATE_ENABLE,"receive/status metadata");
  for(int i=0;i<20;++i)dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));
  dcu.GetActuatorDiagnostics("left",s);
  Check(s.rx_pdo_seq==21&&s.rx_content_seq==content&&s.rx_measurement_seq==measurement&&s.rx_measurement_ns==stamp,"repeated PDO must not refresh measurement change age");
  ActuatorStateSnapshot static_state;dcu.GetActuatorState("left",static_state);
  Check(static_state.pdo_received_ns>first_state.pdo_received_ns,"fresh static PDOs remain publishable despite repeated positions");
  packet.canfd[0][5]^=1;
  dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));
  dcu.GetActuatorDiagnostics("left",s);
  Check(s.rx_content_seq==content+1&&s.rx_measurement_seq==measurement,"heartbeat-only change does not refresh measurement age");
  packet.canfd[1][0]++;
  dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));
  dcu.GetActuatorDiagnostics("left",s);
  Check(s.rx_measurement_seq==measurement,"other channel isolated");
  const auto seq=s.rx_pdo_seq;
  dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&packet),sizeof(packet)-1);
  dcu.GetActuatorDiagnostics("left",s);Check(s.rx_pdo_seq==seq,"short PDO rejected");
  dcu.SetVelocity("left",1);dcu.PullSendData(output.data(),output.size());
  dcu.GetActuatorDiagnostics("left",s);Check(s.tx_format==0&&std::isnan(s.cmd_kp),"administrative frame not decoded as MIT");
  DcuRecvPacket a=packet,b=packet;
  Feedback(a.canfd[0],10000,1000,1000,3);Feedback(b.canfd[0],50000,3000,3000,3);
  dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&a),sizeof(a));ActuatorStateSnapshot av,bv;
  dcu.GetActuatorState("left",av);dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&b),sizeof(b));dcu.GetActuatorState("left",bv);
  std::atomic<bool> done=false;
  std::thread writer([&]{for(int i=0;i<10000;++i){const auto& p=i%2?a:b;dcu.PushRecvData(reinterpret_cast<const uint8_t*>(&p),sizeof(p));}done=true;});
  do {
    ActuatorStateSnapshot v;dcu.GetActuatorState("left",v);
    Check((v.position==av.position&&v.velocity==av.velocity&&v.effort==av.effort)||
          (v.position==bv.position&&v.velocity==bv.velocity&&v.effort==bv.effort),"position/velocity/effort must be one coherent PDO");
  }while(!done);
  writer.join();
  std::cout<<"PASS DCU snapshots, payload age, heartbeat isolation, routing, encoded gains, coherent feedback\n";
}
