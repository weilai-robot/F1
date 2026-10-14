import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
import subprocess

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"scripts"))
import amp_priority_diag as diag


class PriorityDiagnosticsTests(unittest.TestCase):
    def fixture(self,folder):
        files={"cfg/x1_cfg.yaml":"motion_control/install/linux/bin/cfg/x1_cfg.yaml",
               "cfg/x1_cfg_real_nav.yaml":"motion_control/install/linux/bin/cfg/x1_cfg_real_nav.yaml",
               "cfg/control_module/rl_x1.yaml":"motion_control/module/control_module/cfg/rl_x1.yaml",
               "cfg/dcu_driver_module/dcu_x1.yaml":"motion_control/module/dcu_driver_module/cfg/dcu_x1.yaml"}
        for to,source in files.items():
            path=folder/to;path.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/source,path)
        return {name:(folder/name).read_bytes() for name in files}

    def test_prepare_profiles_preserve_source_hardware_and_amp_gains(self):
        for scenario in ["amp","static"]:
            with self.subTest(scenario=scenario),tempfile.TemporaryDirectory() as temp:
                folder=Path(temp);before=self.fixture(folder)
                args=types.SimpleNamespace(bin_dir=folder,scenario=scenario,base_config=None,angle=.1,duration=120,dry_run=False)
                with contextlib.redirect_stdout(io.StringIO()):diag.prepare(args)
                for name,data in before.items():self.assertEqual((folder/name).read_bytes(),data)
                generated=json.loads((folder/"cfg/dcu_driver_module/dcu_priority.yaml").read_text())
                original=diag.load_config(folder/"cfg/dcu_driver_module/dcu_x1.yaml")
                self.assertEqual(generated["dcu_network"],original["dcu_network"])
                self.assertEqual(generated["transmission"],original["transmission"])
                self.assertTrue(generated["diagnostics_conf"]["enabled"])
                manifest=json.loads((folder/"cfg/amp_priority_manifest.json").read_text())
                self.assertEqual(len(manifest["routes"]),29)
                main=json.loads((folder/manifest["main_config"]).read_text())
                driver=next(x for x in main["aimrt"]["module"]["modules"] if x["name"]=="DcuDriverModule")
                self.assertEqual(driver["cfg_file_path"],"./cfg/dcu_driver_module/dcu_priority.yaml")
                if scenario=="static":
                    upper=json.loads((folder/"cfg/control_module/upper_direction_test.json").read_text())
                    self.assertEqual(len(upper["cases"]),17)

    def test_dry_run_writes_nothing(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);self.fixture(folder)
            before={str(p.relative_to(folder)):p.read_bytes() for p in folder.rglob('*') if p.is_file()}
            args=types.SimpleNamespace(bin_dir=folder,scenario="static",base_config=None,angle=.1,duration=120,dry_run=True)
            with contextlib.redirect_stdout(io.StringIO()):diag.prepare(args)
            after={str(p.relative_to(folder)):p.read_bytes() for p in folder.rglob('*') if p.is_file()}
            self.assertEqual(before,after)

    def test_duplicate_route_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);self.fixture(folder);p=folder/"cfg/dcu_driver_module/dcu_x1.yaml"
            d=diag.load_config(p);d["dcu_network"][0]["channel_1"][1]["can_id"]=1;p.write_text(json.dumps(d))
            args=types.SimpleNamespace(bin_dir=folder,scenario="amp",base_config=None,angle=.1,duration=120,dry_run=False)
            with self.assertRaisesRegex(ValueError,"duplicate"):diag.prepare(args)

    def test_frozen_moving_measurement_detected_static_repetition_allowed(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/"dcu.csv";rows=[]
            for i in range(40):
                for name,velocity,ch,format in [("left_shoulder_pitch_actuator",1.,1,1),
                    ("right_shoulder_pitch_actuator",0.,2,1),("unknown",0.,3,0)]:
                    target=.1+i*.002
                    rows.append(dict(timestamp_ns=1_000_000_000_000_000_000+i*20_000_000,actuator=name,
                        ecat_id=1,channel=ch,can_id=1,tx_format=format,rx_seen=1,rx_age_ms=1,
                        rx_content_age_ms=1,rx_measurement_age_ms=i*20,rx_pdo_seq=20*i,
                        rx_measurement_seq=1,tx_slot_enabled=1,error_nibble=0,position=.2,
                        velocity=velocity,effort=3,cmd_position=target if format else float('nan'),
                        cmd_kp=39.9,cmd_kd=1.999,host_effort_max=100,cache_cmd_position=target,
                        cache_cmd_kp=40,cache_cmd_kd=2,host_pd_effort=10,joint_cmd_age_ms=1,
                        rx_hex='0000000000030000',tx_hex='0000000000000000'))
            with p.open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            m=diag.summarize_dcu(p)
            self.assertGreater(m['motors']['left_shoulder_pitch_actuator']['moving_feedback_repetition_fraction'],.5)
            self.assertEqual(m['motors']['right_shoulder_pitch_actuator']['moving_feedback_repetition_fraction'],0)
            self.assertEqual(m['motors']['left_shoulder_pitch_actuator']['measurement_seq_span'],0)
            self.assertGreater(m['motors']['left_shoulder_pitch_actuator']['pdo_seq_span'],0)
            self.assertIsNone(m['motors']['unknown']['target_rom_deg'])
            json.dumps(m,allow_nan=False)
            sliced=diag.summarize_dcu(p,[rows[0]['timestamp_ns'],rows[0]['timestamp_ns']+100_000_000])
            self.assertEqual(sliced['motors']['left_shoulder_pitch_actuator']['moving_feedback_repetition_fraction'],0)

    def test_production_diagnostics_worker_writes_parseable_bounded_csv(self):
        compiler=shutil.which('clang++') or shutil.which('g++')
        if not compiler:self.skipTest('C++ compiler unavailable')
        driver=ROOT/'motion_control/module/dcu_driver_module/src/dcu_driver_module.cc'
        source=driver.read_text()
        body=source[source.index('void DcuDriverModule::DiagnosticsLoop()'):source.rindex('}  // namespace xyber_x1_infer::dcu_driver_module')]
        # Compile the actual worker body with a minimal non-ROS module harness.
        # Production SDK supplies the snapshots; only AimRT logging is stubbed.
        prefix=r'''
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <cstring>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif
#include "internal/dcu.h"
#include "internal/power_flow.h"
#define AIMRT_INFO(...) ((void)0)
#define AIMRT_WARN(...) ((void)0)
namespace xyber_utils {
void FloatToBytes(float f,uint8_t* p){std::memcpy(p,&f,4);}
float BytesToFloat(const uint8_t* p){float f;std::memcpy(&f,p,4);return f;}
}
using namespace xyber;
namespace xyber_x1_infer::dcu_driver_module {
struct DataSpace { struct {double position=0,velocity=0,effort=0,kp=0,kd=0;} cmd,state; };
struct Adapter { Dcu* dcu; bool GetActuatorDiagnostics(const std::string& n,ActuatorDiagnostics& s){return dcu->GetActuatorDiagnostics(n,s);} };
class DcuDriverModule { public:
 void DiagnosticsLoop();
 std::string diagnostics_dir_;double diagnostics_frequency_=50,diagnostics_duration_s_=.03;
 std::atomic<bool> is_running_=true;std::atomic<int64_t> last_joint_cmd_ns_=0;
 std::mutex rw_mtx_;std::vector<std::string> actuator_name_list_;
 std::unordered_map<std::string,DataSpace> actuator_data_space_;
 struct Route{int ecat_id;std::string name;};std::vector<Route> dcu_network_cfg_;
 Adapter* xyber_ctrl_;
};
'''
        suffix=r'''
}
int main(int argc,char** argv){
 using namespace xyber_x1_infer::dcu_driver_module;
 Dcu dcu("body",1);MitParam param=PF_R86_MIT_MODE_DEFAULT_PARAM;
 dcu.RegisterActuator(new xyber_power_flow::PowerFlowR(ActuatorType::POWER_FLOW_R86,"test",1,CtrlChannel::CTRL_CH1,param));
 dcu.SetMitCmd("test",.3,0,0,150,5);
 uint8_t output[sizeof(DcuSendPacket)]{};dcu.PullSendData(output,sizeof(output));
 DcuRecvPacket feedback{};feedback.canfd[0][0]=128;feedback.canfd[0][5]=3;
 dcu.PushRecvData(reinterpret_cast<uint8_t*>(&feedback),sizeof(feedback));
 Adapter adapter{&dcu};DcuDriverModule module;
 module.diagnostics_dir_=argv[1];module.xyber_ctrl_=&adapter;module.actuator_name_list_={"test"};
 module.dcu_network_cfg_.push_back({1,"body"});module.actuator_data_space_["test"].cmd.position=.3;
 module.actuator_data_space_["test"].cmd.kp=150;module.actuator_data_space_["test"].cmd.kd=5;
 module.DiagnosticsLoop();
}
'''
        sdk=ROOT/'motion_control/module/dcu_driver_module/xyber_controller/xyber_api'
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);cpp=folder/'worker.cpp';cpp.write_text(prefix+body+suffix)
            executable=folder/'worker'
            compiled=subprocess.run([compiler,'-std=c++20','-pthread','-fsanitize=undefined','-fno-sanitize-recover=all',
                '-I',str(sdk/'include'),str(cpp),str(sdk/'src/dcu.cpp'),str(sdk/'src/power_flow.cpp'),'-o',str(executable)],capture_output=True,text=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            ran=subprocess.run([str(executable),str(folder/'logs')],capture_output=True,text=True,timeout=5)
            self.assertEqual(ran.returncode,0,ran.stderr)
            logs=list((folder/'logs').glob('dcu_diag_*.csv'));self.assertEqual(len(logs),1)
            with logs[0].open() as f:rows=list(csv.DictReader(f))
            self.assertGreaterEqual(len(rows),1);self.assertLessEqual(len(rows),3)
            self.assertIsNone(rows[0].get(None))
            self.assertEqual(rows[0]['rx_hex'],'8000000000030000')
            self.assertEqual(len(rows[0]['tx_hex']),16)
            self.assertEqual(rows[0]['tx_format'],'1')
            self.assertAlmostEqual(float(rows[0]['cmd_kp']),150,delta=.123)
            self.assertAlmostEqual(float(rows[0]['cmd_kd']),5,delta=.002)


if __name__=='__main__':unittest.main()
